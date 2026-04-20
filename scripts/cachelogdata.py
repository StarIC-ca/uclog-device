#! /usr/bin/env python3

import os
import shutil
import argparse
import hashlib

SIGNATURE_HDR_LEN = 512

def hash(input):
    with open(input, 'rb') as f:
        data = f.read()

    if args.full:
        image_data = data
    else:
        image_data = data[SIGNATURE_HDR_LEN:]
    hash_sha512 = hashlib.sha512()
    hash_sha512.update(image_data)
    return hash_sha512.digest()

if __name__ == '__main__':
    parser = argparse.ArgumentParser('Cache log data')

    parser.add_argument('--bin', help="FW Image binary", required=True)
    parser.add_argument('--logdata', help="Log data to cache", required=True)
    parser.add_argument('-f', '--full', action='store_true', help="Use full file contents")
    parser.add_argument('--hashfile', help="File with hash", default="")

    args = parser.parse_args()

    if not os.path.isfile(args.bin):
        print("Image binary not found")
        exit(1)
    if not os.path.isfile(args.logdata):
        print("Logdata file not found")
        exit(1)

    if args.hashfile != "":
        if not os.path.isfile(args.hashfile):
            print("Hash file not found")
            exit(1)
        with open(args.hashfile, 'rb') as f:
            app_hash = f.read()
    else:
        app_hash = hash(args.bin)

    user_dir = os.path.expanduser("~")
    cache_dir = os.path.join(user_dir, ".cache", "uclog")
    if not os.path.exists(cache_dir):
        os.makedirs(cache_dir)
    shutil.copy(args.logdata, os.path.join(cache_dir, app_hash.hex() + ".logdata"))
