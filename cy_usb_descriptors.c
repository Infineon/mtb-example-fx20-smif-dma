/***************************************************************************//**
 * \file cy_usb_descriptors.c
 * \version 1.0
 *
 * Defines the USB descriptors used in FX SMIF DMA example
 *
 *******************************************************************************
 * \copyright
 * (c) (2023-2025), Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *******************************************************************************/


#include "cy_pdl.h"
#include "cy_usb_app.h"


/* Standard device descriptor for USB 3.2 */
USB3_DESC_ATTRIBUTES uint8_t CyFxUSB30DeviceDscr[] =
{
  0x12,                           /* Descriptor size */
  0x01,                           /* Device descriptor type */
  0x20,0x03,                      /* USB 3.2 */
  0x00,                           /* Device class */
  0x00,                           /* Device Sub-class */
  0x00,                           /* Device protocol */
  0x09,                           /* Maxpacket size for EP0 : 2^9 */
  0xB4,0x04,                      /* Vendor ID */
#if USE_WINUSB
    0xF1,0xF6,                      /* Product ID */
#else
    0xF1,0x00,                      /* Product ID */
#endif /* USE_WINUSB */
  0x00,0x00,                      /* Device release number */
  0x01,                           /* Manufacture string index */
  0x02,                           /* Product string index */
  0x00,                           /* Serial number string index */
  0x01                            /* Number of configurations */
};

/* Standard device descriptor */
USB3_DESC_ATTRIBUTES uint8_t CyFxUSB20DeviceDscr[] =
{
  0x12,                           /* Descriptor size */
  0x01,                           /* Device descriptor type */
  0x00,0x02,                      /* USB 2.00 */
  0x00,                           /* Device class */
  0x00,                           /* Device sub-class */
  0x00,                           /* Device protocol */
  0x40,                           /* Maxpacket size for EP0 : 64 bytes */
  0xB4,0x04,                      /* Vendor ID */
#if USE_WINUSB
    0xF1,0xF6,                      /* Product ID */
#else
    0xF1,0x00,                      /* Product ID */
#endif /* USE_WINUSB */
  0x00,0x00,                      /* Device release number */
  0x01,                           /* Manufacture string index */
  0x02,                           /* Product string index */
  0x00,                           /* Serial number string index */
  0x01                            /* Number of configurations */
};

/* Binary device object store descriptor */
USB3_DESC_ATTRIBUTES uint8_t CyFxUSBBOSDscr[] =
{
  0x05,                           /* Descriptor size */
  0x0F,                           /* Device descriptor type */
  0x16,0x00,                      /* Length of this descriptor and all sub descriptors */
  0x02,                           /* Number of device capability descriptors */

  /* USB 2.0 Extension */
  0x07,                           /* Descriptor size */
  0x10,                           /* Device capability type descriptor */
  0x02,                           /* USB 2.0 extension capability type */
  0x1E,0x64,0x00,0x00,            /* Supported device level features: LPM support, BESL supported,
                                     Baseline BESL=400 us, Deep BESL=1000 us. */

  /* SuperSpeed Device Capability */
  0x0A,                           /* Descriptor size */
  0x10,                           /* Device capability type descriptor */
  0x03,                           /* SuperSpeed device capability type */
  0x00,                           /* Supported device level features  */
  0x0E,0x00,                      /* Speeds supported by the device : SS, HS and FS */
  0x03,                           /* Functionality support */
  0x00,                           /* U1 device exit latency */
  0x00,0x00                       /* U2 device exit latency */
};

/* Standard device qualifier descriptor */
USB3_DESC_ATTRIBUTES uint8_t CyFxUSBDeviceQualDscr[] =
{
  0x0A,                           /* descriptor size */
  0x06,                           /* Device qualifier descriptor type */
  0x00,0x02,                      /* USB 2.0 */
  0x00,                           /* Device class */
  0x00,                           /* Device sub-class */
  0x00,                           /* Device protocol */
  0x40,                           /* Maxpacket size for EP0 : 64 bytes */
  0x01,                           /* Number of configurations */
  0x00                            /* Reserved */
};

/* Standard super speed configuration descriptor */
USB3_DESC_ATTRIBUTES uint8_t CyFxUSBSSConfigDscr[] =
{
  /* Configuration descriptor */
  0x09,                           /* Descriptor size */
  0x02,                           /* Configuration descriptor type */
  0x12,0x00,                      /* Length of this descriptor and all sub descriptors */
  0x01,                           /* Number of interfaces */
  0x01,                           /* Configuration number */
  0x00,                           /* Configuration string index */
  0x80,                           /* Config characteristics - bus powered without remote wake support. */
  0x32,                           /* Max power consumption of device (in 8mA unit) : 400mA */

  /* Interface descriptor */
  0x09,                           /* Descriptor size */
  0x04,                           /* Interface Descriptor type */
  0x00,                           /* Interface number */
  0x00,                           /* Alternate setting number */
  0x00,                           /* Number of end points */
  0xFF,                           /* Interface class */
  0x00,                           /* Interface sub class */
  0x00,                           /* Interface protocol code */
  0x00,                            /* Interface descriptor string index */
};

/* Standard High Speed Configuration Descriptor */
USB3_DESC_ATTRIBUTES uint8_t CyFxUSBHSConfigDscr[] =
{
  /* Configuration descriptor */
  0x09,                           /* Descriptor size */
  0x02,                           /* Configuration descriptor type */
  0x12, 0x00,                     /* Length of this descriptor and all sub descriptors */
  0x01,                           /* Number of interfaces */
  0x01,                           /* Configuration number */
  0x00,                           /* Configuration string index */
  0x80,                           /* Config characteristics - bus powered */
  0x32,                           /* Max power consumption of device (in 2mA unit) : 100mA */

  /* Interface descriptor */
  0x09,                           /* Descriptor size */
  0x04,                           /* Interface Descriptor type */
  0x00,                           /* Interface number */
  0x00,                           /* Alternate setting number */
  0x00,                           /* Number of end points */
  0xFF,                           /* Interface class */
  0x00,                           /* Interface sub class */
  0x00,                           /* Interface protocol code */
  0x00,                            /* Interface descriptor string index */
};

/* Standard full speed configuration descriptor : full speed is not supported. */
USB3_DESC_ATTRIBUTES uint8_t CyFxUSBFSConfigDscr[] =
{
  /* Configuration descriptor */
  0x09,                           /* Descriptor size */
  0x02,                           /* Configuration descriptor type */
  0x12, 0x00,                     /* Length of this descriptor and all sub descriptors */
  0x01,                           /* Number of interfaces */
  0x01,                           /* Configuration number */
  0x00,                           /* Configuration string index */
  0x80,                           /* Config characteristics - bus powered */
  0x32,                           /* Max power consumption of device (in 2mA unit) : 100mA */

  /* Interface descriptor */
  0x09,                           /* Descriptor size */
  0x04,                           /* Interface Descriptor type */
  0x00,                           /* Interface number */
  0x00,                           /* Alternate setting number */
  0x00,                           /* Number of end points */
  0xFF,                           /* Interface class */
  0x00,                           /* Interface sub class */
  0x00,                           /* Interface protocol code */
  0x00,                            /* Interface descriptor string index */
};

/* Standard language ID string descriptor */
USB3_DESC_ATTRIBUTES uint8_t CyFxUSBStringLangIDDscr[] =
{
  0x04,                           /* Descriptor size */
  0x03,                           /* Device descriptor type */
  0x09,0x04                       /* Language ID supported */
};

/* Standard manufacturer string descriptor */
USB3_DESC_ATTRIBUTES uint8_t CyFxUSBManufactureDscr[] =
{
  0x12,        /* Descriptor size */
  0x03,        /* Device descriptor type */
  'I',0x00,
  'n',0x00,
  'f',0x00,
  'i',0x00,
  'n',0x00,
  'e',0x00,
  'o',0x00,
  'n',0x00
};

/* Standard product string descriptor */
USB3_DESC_ATTRIBUTES uint8_t CyFxUSBProductDscr[] =
{
  0x26,        /* Descriptor Size */
  0x03,        /* Device descriptor type */
  'F',0x00,
  'X',0x00,
  ' ',0x00,
  'S',0x00,
  'M',0x00,
  'I',0x00,
  'F',0x00,
  ' ',0x00,
  'P',0x00,
  'R',0x00,
  'O',0x00,
  'G',0x00,
  'R',0x00,
  'A',0x00,
  'M',0x00,
  'M',0x00,
  'E',0x00,
  'R',0x00
};
/* MS OS String Descriptor */
USB3_DESC_ATTRIBUTES uint8_t glOsString[] =
{
    0x12, /* Length. */
    0x03, /* Type - string. */
    'M', 0x00, 'S', 0x00, 'F', 0x00, 'T', 0x00, '1', 0x00, '0', 0x00, '0', 0x00, /* Signature. */
    MS_VENDOR_CODE, /* MS vendor code. */
    0x00 /* Padding. */
};
USB3_DESC_ATTRIBUTES uint8_t glOsCompatibilityId[] =
{
    /* Header */
    0x28, 0x00, 0x00, 0x00, /* length Need to be updated based on number of interfaces. */
    0x00, 0x01, /* BCD version */
    0x04, 0x00, /* Index: 4 - compatibility ID */
    0x01, /* count. Need to be updated based on number of interfaces. */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* reserved. */
    /* First Interface */
    0x00, /* Interface number */
    0x01, /* reserved: Need to be 1. */
    0x57, 0x49, 0x4E, 0x55, 0x53, 0x42, 0x00, 0x00, /* comp ID to bind the device with
                                                       WinUSB.*/
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* sub-compatibility ID - NONE. */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* reserved - needs to be zero. */
};
USB3_DESC_ATTRIBUTES uint8_t glOsFeature[] =
{
    /* Header */
    0x8E, 0x00, 0x00, 0x00, /* Length. */
    0x00, 0x01, /* BCD version. 1.0 as per MS */
    0x05, 0x00, /* Index */
    0x01, 0x00, /* count. */
    /* Property section. */
    0x84, 0x00, 0x00, 0x00, /* length */
    0x01, 0x00, 0x00, 0x00, /* dwPropertyDataType: REG_DWORD_LITTLE_ENDIAN */
    0x28, 0x00, /* wPropertyNameLength: 0x30 */

    0x44, 0x00, 0x65, 0x00, 0x76, 0x00, 0x69, 0x00, 0x63, 0x00, 0x65, 0x00, 0x49, 0x00, 0x6E, 0x00,
    0x74, 0x00, 0x65, 0x00, 0x72, 0x00, 0x66, 0x00, 0x61, 0x00, 0x63, 0x00, 0x65, 0x00, 0x47, 0x00,
    0x55, 0x00, 0x49, 0x00, 0x44, 0x00, 0x00, 0x00, /* bPropertyName: DeviceInterfaceGUID */
    0x4E, 0x00, 0x00, 0x00, /* dwPropertyDataLength: 4E */

    '{', 0x00, '0', 0x00, '1', 0x00, '2', 0x00, '3', 0x00, '4', 0x00, '5', 0x00, '6', 0x00,
    '7', 0x00, '-', 0x00, '2', 0x00, 'A', 0x00, '4', 0x00, 'F', 0x00, '-', 0x00, '4', 0x00,
    '9', 0x00, 'E', 0x00, 'E', 0x00, '-', 0x00, '8', 0x00, 'D', 0x00, 'D', 0x00, '3', 0x00,
    '-', 0x00, 'F', 0x00, 'A', 0x00, 'D', 0x00, 'E', 0x00, 'A', 0x00, '3', 0x00, '7', 0x00,
    '7', 0x00, '2', 0x00, '3', 0x00, '4', 0x00, 'A', 0x00, '}', 0x00, 0x00, 0x00
        /* bPropertyData: {01234567-2A4F-49EE-8DD3-FADEA377234A} */
};
/*[]*/

