/**
  Qualcomm's EFI_USB_DEVICE_PROTOCOL on top of the UEFI standard EFI_USBFN_IO_PROTOCOL.

  HakiOS's xnuloader serves fastboot through Qualcomm's polled USB device protocol, which on Snapdragon
  phones the vendor's own UsbDeviceDxe provides. On other SoCs (Samsung Exynos: SamsungPkg's UsbfnDwc3Dxe)
  only EFI_USBFN_IO_PROTOCOL exists; this driver gives the loader the interface it expects:

    StartEx                  starts the controller and enables the endpoints of configuration 1, interface 0
    HandleEvent              polls the controller; answers endpoint 0 itself from the descriptor set given to
                             StartEx (GET_DESCRIPTOR, GET_STATUS, GET/SET_CONFIGURATION, ...), reports
                             SET_CONFIGURATION as "connected", a reset or detach as "disconnected", and the
                             completion of every other transfer by its endpoint address (0x81 IN, 0x01 OUT)
    Send / AbortXfer         queue / cancel a transfer on an endpoint address
    Allocate/FreeTransferBuffer, Stop, SetEndpointStallState   passed through

  Only what xnuloader uses is meant to be complete: one configuration, one interface, bulk endpoints,
  high speed. The interface follows the open-source ABL's QcomModulePkg/Include/Protocol/EFIUsbDevice.h
  (Mu-Silicium's QcomPkg has the same header); its layout is restated here so that no Qualcomm package is needed.

  SPDX-License-Identifier: BSD-2-Clause-Patent
**/
#include <Uefi.h>

#include <IndustryStandard/Usb.h>

#include <Library/BaseMemoryLib.h>
#include <Library/DebugLib.h>
#include <Library/UefiBootServicesTableLib.h>

#include <Protocol/UsbFunctionIo.h>

//
// Qualcomm EFI_USB_DEVICE_PROTOCOL {D9D9CE48-44B8-4F49-8E3E-2A3B927DC6C1}
//
STATIC EFI_GUID mUsbDeviceProtocolGuid = {
  0xd9d9ce48, 0x44b8, 0x4f49, { 0x8e, 0x3e, 0x2a, 0x3b, 0x92, 0x7d, 0xc6, 0xc1 }
};

#define USB_DEVICE_PROTOCOL_REVISION  0x000000010000001ULL

typedef enum {
  UsbDeviceEventNoEvent,
  UsbDeviceEventDeviceStateChange,
  UsbDeviceEventTransferNotification,
  UsbDeviceEventOemEvent
} USB_DEVICE_EVENT;

typedef enum {
  UsbDeviceTransferStatusActive,
  UsbDeviceTransferStatusCompleteOK,
  UsbDeviceTransferStatusCancelled,
  UsbDeviceTransferStatusCompleteError
} USB_DEVICE_TRANSFER_STATUS;

typedef enum {
  UsbDeviceStateConnected,
  UsbDeviceStateDisconnected
} USB_DEVICE_STATE;

typedef struct {
  USB_DEVICE_TRANSFER_STATUS Status;
  UINT8                      EndpointIndex;
  UINTN                      BytesCompleted;
  VOID                       *DataBuffer;
} USB_DEVICE_TRANSFER_OUTCOME;

typedef struct {
  EFI_GUID DataGuid;
  UINTN    VersionNumber;
  UINTN    Size;
  VOID     *Context;
} USB_DEVICE_OEM_DATA;

typedef union {
  USB_DEVICE_STATE            DeviceState;
  USB_DEVICE_TRANSFER_OUTCOME TransferOutcome;
  USB_DEVICE_OEM_DATA         OemData;
} USB_DEVICE_EVENT_DATA;

typedef struct {
  EFI_USB_DEVICE_DESCRIPTOR            *DeviceDescriptor;
  VOID                                 **Descriptors;
  EFI_USB_DEVICE_DESCRIPTOR            *SSDeviceDescriptor;
  VOID                                 **SSDescriptors;
  VOID                                 *DeviceQualifierDescriptor;   // 10 bytes (USB 2.0, 9.6.2)
  VOID                                 *BinaryDeviceOjectStore;
  UINT8                                StringDescriptorCount;
  EFI_USB_STRING_DESCRIPTOR            **StringDescritors;
} USB_DEVICE_DESCRIPTOR_SET;

typedef struct {
  UINT64     Revision;
  VOID       *Start;                       // the older Start(): not provided, StartEx is
  EFI_STATUS (EFIAPI *Send)(UINT8 EndpointIndex, UINTN Size, VOID *Buffer);
  EFI_STATUS (EFIAPI *HandleEvent)(USB_DEVICE_EVENT *Event, UINTN *DataSize, USB_DEVICE_EVENT_DATA *EventData);
  EFI_STATUS (EFIAPI *AllocateTransferBuffer)(UINTN Size, VOID **Buffer);
  EFI_STATUS (EFIAPI *FreeTransferBuffer)(VOID *Buffer);
  EFI_STATUS (EFIAPI *Stop)(VOID);
  EFI_STATUS (EFIAPI *AbortXfer)(UINT8 EndpointIndex);
  EFI_STATUS (EFIAPI *SetEndpointStallState)(UINT8 EndpointIndex, BOOLEAN State);
  EFI_STATUS (EFIAPI *StartEx)(USB_DEVICE_DESCRIPTOR_SET *DescriptorSet);
} USB_DEVICE_PROTOCOL;

#define USB_REQ_TYPE_MASK     (0x03 << 5)       // bmRequestType bits 6:5 (MdePkg has the values, not the mask)
#define MAX_BRIDGE_ENDPOINTS  8
#define EP0_BUFFER_SIZE       SIZE_4KB

STATIC EFI_USBFN_IO_PROTOCOL       *mUsbfn;
STATIC USB_DEVICE_DESCRIPTOR_SET    mSet;
STATIC VOID                        *mEp0Buffer;
STATIC UINT8                        mConfiguration;
STATIC BOOLEAN                      mStarted;

STATIC EFI_USB_INTERFACE_INFO       mInterfaceInfo;
STATIC EFI_USB_INTERFACE_INFO      *mInterfaceTable[1];
STATIC EFI_USB_CONFIG_INFO          mConfigInfo;
STATIC EFI_USB_CONFIG_INFO         *mConfigTable[1];
STATIC EFI_USB_DEVICE_INFO          mDeviceInfo;
STATIC EFI_USB_ENDPOINT_DESCRIPTOR *mEndpoints[MAX_BRIDGE_ENDPOINTS];

STATIC
EFI_USBFN_ENDPOINT_DIRECTION
EndpointDirection (IN UINT8 EndpointAddress)
{
  return (EndpointAddress & USB_ENDPOINT_DIR_IN) ? EfiUsbEndpointDirectionDeviceTx : EfiUsbEndpointDirectionDeviceRx;
}

STATIC
UINT16
DescriptorTotalLength (IN CONST VOID *Descriptor)
{
  CONST UINT8 *Bytes = Descriptor;

  return (UINT16)(Bytes[2] | (Bytes[3] << 8));
}

//
// The configuration descriptor of configuration 1 as one blob (USB 2.0, 9.4.3): its first interface and the
// endpoints that follow it become the EFI_USB_DEVICE_INFO the USBFN driver configures.
//
STATIC
EFI_STATUS
BuildDeviceInfo (VOID)
{
  UINT8  *Config;
  UINT16 Total;
  UINT16 Offset;
  UINT8  Count = 0;

  if (mSet.DeviceDescriptor == NULL || mSet.Descriptors == NULL || mSet.Descriptors[0] == NULL) {
    return EFI_INVALID_PARAMETER;
  }

  Config = mSet.Descriptors[0];
  Total  = DescriptorTotalLength (Config);
  if (Config[1] != USB_DESC_TYPE_CONFIG || Total < sizeof (EFI_USB_CONFIG_DESCRIPTOR)) {
    return EFI_INVALID_PARAMETER;
  }

  ZeroMem (&mInterfaceInfo, sizeof (mInterfaceInfo));

  for (Offset = Config[0]; Offset + 2 <= Total && Config[Offset] != 0; Offset += Config[Offset]) {
    UINT8 Type = Config[Offset + 1];

    if (Type == USB_DESC_TYPE_INTERFACE) {
      if (mInterfaceInfo.InterfaceDescriptor != NULL) {
        break;                                      // only the first interface
      }
      mInterfaceInfo.InterfaceDescriptor = (EFI_USB_INTERFACE_DESCRIPTOR *)&Config[Offset];
    } else if (Type == USB_DESC_TYPE_ENDPOINT && mInterfaceInfo.InterfaceDescriptor != NULL) {
      if (Count == MAX_BRIDGE_ENDPOINTS) {
        return EFI_UNSUPPORTED;
      }
      mEndpoints[Count++] = (EFI_USB_ENDPOINT_DESCRIPTOR *)&Config[Offset];
    }
  }

  if (mInterfaceInfo.InterfaceDescriptor == NULL || Count != mInterfaceInfo.InterfaceDescriptor->NumEndpoints) {
    return EFI_INVALID_PARAMETER;
  }

  mInterfaceInfo.EndpointDescriptorTable = mEndpoints;
  mInterfaceTable[0]                     = &mInterfaceInfo;
  mConfigInfo.ConfigDescriptor           = (EFI_USB_CONFIG_DESCRIPTOR *)Config;
  mConfigInfo.InterfaceInfoTable         = mInterfaceTable;
  mConfigTable[0]                        = &mConfigInfo;
  mDeviceInfo.DeviceDescriptor           = mSet.DeviceDescriptor;
  mDeviceInfo.ConfigInfoTable            = mConfigTable;

  return EFI_SUCCESS;
}

STATIC
VOID
Ep0Stall (VOID)
{
  BOOLEAN Stall = TRUE;

  mUsbfn->SetEndpointStallState (mUsbfn, 0, EfiUsbEndpointDirectionDeviceTx, &Stall);
}

STATIC
VOID
Ep0Reply (
  IN CONST VOID             *Data,
  IN UINTN                   Length,
  IN EFI_USB_DEVICE_REQUEST *Request)
{
  UINTN Size;

  if (Data == NULL) {
    Ep0Stall ();
    return;
  }

  Size = MIN (MIN (Length, (UINTN)Request->Length), (UINTN)EP0_BUFFER_SIZE);
  CopyMem (mEp0Buffer, Data, Size);

  if (EFI_ERROR (mUsbfn->Transfer (mUsbfn, 0, EfiUsbEndpointDirectionDeviceTx, &Size, mEp0Buffer))) {
    Ep0Stall ();
  }
}

STATIC
VOID
Ep0GetDescriptor (IN EFI_USB_DEVICE_REQUEST *Request)
{
  UINT8      Type  = (UINT8)(Request->Value >> 8);
  UINT8      Index = (UINT8)(Request->Value & 0xFF);
  CONST VOID *Data = NULL;
  UINTN      Length = 0;

  switch (Type) {
    case USB_DESC_TYPE_DEVICE:
      Data   = mSet.DeviceDescriptor;
      Length = sizeof (EFI_USB_DEVICE_DESCRIPTOR);
      break;

    case USB_DESC_TYPE_CONFIG:
      if (Index == 0) {
        Data   = mSet.Descriptors[0];
        Length = DescriptorTotalLength (Data);
      }
      break;

    case USB_DESC_TYPE_STRING:
      if (mSet.StringDescritors != NULL && Index < mSet.StringDescriptorCount && mSet.StringDescritors[Index] != NULL) {
        Data   = mSet.StringDescritors[Index];
        Length = mSet.StringDescritors[Index]->Length;
      }
      break;

    case 0x06:  // DEVICE_QUALIFIER
      Data   = mSet.DeviceQualifierDescriptor;
      Length = (Data != NULL) ? 10 : 0;
      break;

    case 0x0F:  // BOS
      Data   = mSet.BinaryDeviceOjectStore;
      Length = (Data != NULL) ? DescriptorTotalLength (Data) : 0;
      break;

    default:
      break;
  }

  Ep0Reply (Data, Length, Request);
}

//
// Endpoint 0. Returns TRUE when the request makes the device configured (reported as "connected").
//
STATIC
BOOLEAN
Ep0Setup (IN EFI_USB_DEVICE_REQUEST *Request)
{
  STATIC CONST UINT8 Zero[2] = { 0, 0 };
  BOOLEAN            Clear   = FALSE;

  if ((Request->RequestType & USB_REQ_TYPE_MASK) != USB_REQ_TYPE_STANDARD) {
    Ep0Stall ();
    return FALSE;
  }

  switch (Request->Request) {
    case USB_REQ_GET_DESCRIPTOR:
      Ep0GetDescriptor (Request);
      return FALSE;

    case USB_REQ_GET_STATUS:
      Ep0Reply (Zero, sizeof (Zero), Request);
      return FALSE;

    case USB_REQ_GET_CONFIG:
      Ep0Reply (&mConfiguration, 1, Request);
      return FALSE;

    case USB_REQ_GET_INTERFACE:
      Ep0Reply (Zero, 1, Request);
      return FALSE;

    case USB_REQ_SET_CONFIG:
      mConfiguration = (UINT8)Request->Value;       // the status stage is the USBFN driver's
      return mConfiguration != 0;

    case USB_REQ_CLEAR_FEATURE:
      if ((Request->RequestType & 0x1F) == USB_TARGET_ENDPOINT && Request->Value == 0 /* ENDPOINT_HALT */) {
        UINT8 Address = (UINT8)Request->Index;
        mUsbfn->SetEndpointStallState (mUsbfn, Address & 0x0F, EndpointDirection (Address), &Clear);
      }
      return FALSE;

    case USB_REQ_SET_INTERFACE:
    case USB_REQ_SET_FEATURE:
      return FALSE;

    default:
      Ep0Stall ();
      return FALSE;
  }
}

STATIC
EFI_STATUS
EFIAPI
BridgeHandleEvent (
  OUT USB_DEVICE_EVENT      *Event,
  OUT UINTN                 *DataSize,
  OUT USB_DEVICE_EVENT_DATA *EventData)
{
  EFI_STATUS                 Status;
  EFI_USBFN_MESSAGE          Message;
  EFI_USBFN_MESSAGE_PAYLOAD  Payload;
  UINTN                      PayloadSize = sizeof (Payload);
  EFI_USBFN_TRANSFER_RESULT *Result;
  BOOLEAN                    WasConfigured;

  if (Event == NULL || DataSize == NULL || EventData == NULL) {
    return EFI_INVALID_PARAMETER;
  }

  *Event    = UsbDeviceEventNoEvent;
  *DataSize = 0;

  if (!mStarted) {
    return EFI_NOT_READY;
  }

  Status = mUsbfn->EventHandler (mUsbfn, &Message, &PayloadSize, &Payload);
  if (EFI_ERROR (Status)) {
    return Status;
  }

  switch (Message) {
    case EfiUsbMsgSetupPacket:
      if (Ep0Setup (&Payload.udr)) {
        *Event                 = UsbDeviceEventDeviceStateChange;
        *DataSize              = sizeof (USB_DEVICE_STATE);
        EventData->DeviceState = UsbDeviceStateConnected;
      }
      break;

    case EfiUsbMsgEndpointStatusChangedRx:
    case EfiUsbMsgEndpointStatusChangedTx:
      Result = &Payload.utr;
      if (Result->EndpointIndex == 0) {
        break;                                      // our own endpoint 0 data stage
      }

      *Event    = UsbDeviceEventTransferNotification;
      *DataSize = sizeof (USB_DEVICE_TRANSFER_OUTCOME);
      EventData->TransferOutcome.EndpointIndex  = (UINT8)(Result->EndpointIndex |
                                                  (Result->Direction == EfiUsbEndpointDirectionDeviceTx ? USB_ENDPOINT_DIR_IN : 0));
      EventData->TransferOutcome.BytesCompleted = Result->BytesTransferred;
      EventData->TransferOutcome.DataBuffer     = Result->Buffer;
      switch (Result->TransferStatus) {
        case UsbTransferStatusComplete:
          EventData->TransferOutcome.Status = UsbDeviceTransferStatusCompleteOK;
          break;
        case UsbTransferStatusAborted:
          EventData->TransferOutcome.Status = UsbDeviceTransferStatusCancelled;
          break;
        case UsbTransferStatusActive:
          EventData->TransferOutcome.Status = UsbDeviceTransferStatusActive;
          break;
        default:
          EventData->TransferOutcome.Status = UsbDeviceTransferStatusCompleteError;
          break;
      }
      break;

    case EfiUsbMsgBusEventReset:
    case EfiUsbMsgBusEventDetach:
      WasConfigured  = (mConfiguration != 0);
      mConfiguration = 0;
      if (WasConfigured) {
        *Event                 = UsbDeviceEventDeviceStateChange;
        *DataSize              = sizeof (USB_DEVICE_STATE);
        EventData->DeviceState = UsbDeviceStateDisconnected;
      }
      break;

    default:
      break;
  }

  return EFI_SUCCESS;
}

STATIC
EFI_STATUS
EFIAPI
BridgeSend (
  IN UINT8 EndpointAddress,
  IN UINTN Size,
  IN VOID  *Buffer)
{
  if (!mStarted || (EndpointAddress & 0x0F) == 0) {
    return EFI_INVALID_PARAMETER;
  }

  return mUsbfn->Transfer (mUsbfn, EndpointAddress & 0x0F, EndpointDirection (EndpointAddress), &Size, Buffer);
}

STATIC
EFI_STATUS
EFIAPI
BridgeAbortXfer (IN UINT8 EndpointAddress)
{
  if (!mStarted) {
    return EFI_NOT_READY;
  }

  return mUsbfn->AbortTransfer (mUsbfn, EndpointAddress & 0x0F, EndpointDirection (EndpointAddress));
}

STATIC
EFI_STATUS
EFIAPI
BridgeSetEndpointStallState (
  IN UINT8   EndpointAddress,
  IN BOOLEAN State)
{
  if (!mStarted) {
    return EFI_NOT_READY;
  }

  return mUsbfn->SetEndpointStallState (mUsbfn, EndpointAddress & 0x0F, EndpointDirection (EndpointAddress), &State);
}

STATIC
EFI_STATUS
EFIAPI
BridgeAllocateTransferBuffer (
  IN  UINTN Size,
  OUT VOID  **Buffer)
{
  if (mUsbfn == NULL) {
    return EFI_NOT_READY;
  }

  return mUsbfn->AllocateTransferBuffer (mUsbfn, Size, Buffer);
}

STATIC
EFI_STATUS
EFIAPI
BridgeFreeTransferBuffer (IN VOID *Buffer)
{
  if (mUsbfn == NULL) {
    return EFI_NOT_READY;
  }

  return mUsbfn->FreeTransferBuffer (mUsbfn, Buffer);
}

STATIC
EFI_STATUS
EFIAPI
BridgeStop (VOID)
{
  if (!mStarted) {
    return EFI_SUCCESS;
  }

  mStarted       = FALSE;
  mConfiguration = 0;

  return mUsbfn->StopController (mUsbfn);
}

STATIC
EFI_STATUS
EFIAPI
BridgeStartEx (IN USB_DEVICE_DESCRIPTOR_SET *DescriptorSet)
{
  EFI_STATUS Status;

  if (DescriptorSet == NULL) {
    return EFI_INVALID_PARAMETER;
  }

  if (mStarted) {
    return EFI_ALREADY_STARTED;
  }

  if (mUsbfn == NULL) {
    Status = gBS->LocateProtocol (&gEfiUsbFunctionIoProtocolGuid, NULL, (VOID **)&mUsbfn);
    if (EFI_ERROR (Status)) {
      DEBUG ((EFI_D_ERROR, "UsbDeviceBridge: no USBFN I/O protocol! Status = %r\n", Status));
      return Status;
    }
  }

  CopyMem (&mSet, DescriptorSet, sizeof (mSet));

  Status = BuildDeviceInfo ();
  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_ERROR, "UsbDeviceBridge: unusable descriptor set! Status = %r\n", Status));
    return Status;
  }

  if (mEp0Buffer == NULL) {
    Status = mUsbfn->AllocateTransferBuffer (mUsbfn, EP0_BUFFER_SIZE, &mEp0Buffer);
    if (EFI_ERROR (Status)) {
      return Status;
    }
  }

  Status = mUsbfn->StartController (mUsbfn);
  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_ERROR, "UsbDeviceBridge: StartController failed! Status = %r\n", Status));
    return Status;
  }

  Status = mUsbfn->ConfigureEnableEndpoints (mUsbfn, &mDeviceInfo);
  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_ERROR, "UsbDeviceBridge: ConfigureEnableEndpoints failed! Status = %r\n", Status));
    mUsbfn->StopController (mUsbfn);
    return Status;
  }

  mConfiguration = 0;
  mStarted       = TRUE;

  return EFI_SUCCESS;
}

STATIC USB_DEVICE_PROTOCOL mUsbDevice = {
  USB_DEVICE_PROTOCOL_REVISION,
  NULL,
  BridgeSend,
  BridgeHandleEvent,
  BridgeAllocateTransferBuffer,
  BridgeFreeTransferBuffer,
  BridgeStop,
  BridgeAbortXfer,
  BridgeSetEndpointStallState,
  BridgeStartEx
};

EFI_STATUS
EFIAPI
UsbDeviceBridgeDxeInitialize (
  IN EFI_HANDLE        ImageHandle,
  IN EFI_SYSTEM_TABLE *SystemTable)
{
  EFI_HANDLE Handle = NULL;

  return gBS->InstallMultipleProtocolInterfaces (&Handle, &mUsbDeviceProtocolGuid, &mUsbDevice, NULL);
}
