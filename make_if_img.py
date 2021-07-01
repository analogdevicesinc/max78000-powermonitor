#
# Derived from "post_build_script.py" which carries the following copyright.
#

#
# DAPLink Interface Firmware
# Copyright (c) 2009-2019, ARM Limited, All Rights Reserved
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may
# not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

import argparse
import struct
import binascii

# Magic constants pulled from the max32630 interface image
DEFAULT_BUILD_KEY = "9B939E8F"
DEFAULT_HIC_ID = "97969906"
DEFAULT_VERSION = "000000FE"

VECTOR_FMT = "<7I"
CHECKSUM_FMT = "<1I"
CHECKSUM_OFFS = 0x1C
BUILD_KEY_OFFS = 0x20
HIC_ID_OFFS = 0x24
VERSION_OFFS = 0x28


def str2ui32(str_val, name):
    """
        Convert string to unsigned 32-bit int
    """
    uint_val = int(str_val, 16)
    assert -1 < uint_val < 2**32, "assert -1 < %s < 2^32 failed" % name
    return uint_val

def compute_vector_checksum(image):
    """
        Compute checksum of the first 7 vectors
    """
    vector_size = struct.calcsize(VECTOR_FMT)
    vector_data = image[0:vector_size]

    vectors = struct.unpack(VECTOR_FMT, vector_data)

    chksum = 0
    for vector in vectors:
        chksum += vector

    return (~chksum + 1) & 0xFFFFFFFF  # Two's compliment


def fixup_if_img(input_file, output_file, build_key, hic_id, version):
    """
    ...
    """
    with open(input_file, "rb") as ifile:
        image = bytearray(ifile.read())

    assert len(image) > (4 * 256), "Input file does not seem reasonable at %i bytes" % len(image)

    # Compute checksum and poke into image
    vector_chksum = compute_vector_checksum(image)
    packed_chksum = struct.pack(CHECKSUM_FMT, vector_chksum)
    image[CHECKSUM_OFFS:CHECKSUM_OFFS+4] = packed_chksum

    # Poke build_key, hic_id and version into image
    image[BUILD_KEY_OFFS:BUILD_KEY_OFFS+4] = struct.pack("<1I", str2ui32(build_key, "build_key"))
    image[HIC_ID_OFFS:HIC_ID_OFFS+4] = struct.pack("<1I", str2ui32(hic_id, "hic_id"))
    image[VERSION_OFFS:VERSION_OFFS+4] = struct.pack("<1I", str2ui32(version, "version"))

    # Compute 32-bit CRC over entire image sans location of CRC and poke into image
    crc_len = len(image) - 4
    crc = binascii.crc32(image[0:crc_len]) & 0xFFFFFFFF
    image[crc_len:crc_len+4] = struct.pack("<1I", crc)

    with open(output_file, "wb") as ofile:
        ofile.write(image)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Make DAPLink bootloader image')
    parser.add_argument("input", help="Bin file to read from.")
    parser.add_argument("output", help="Output file name to write.")
    parser.add_argument("--build-key", metavar="<32-bit hex>", default=DEFAULT_BUILD_KEY,
                        help="build key")
    parser.add_argument("--hic-id", metavar="<32-bit hex>",  default=DEFAULT_HIC_ID,
                        help="hic id")
    parser.add_argument("--version", metavar="<32-bit hex>", default=DEFAULT_VERSION,
                        help="version")
    args = parser.parse_args()
    fixup_if_img(args.input, args.output, args.build_key, args.hic_id, args.version)
