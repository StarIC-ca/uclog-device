#! /usr/bin/env python3

# © 2025 Unit Circle Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Example usage:
# ./sbl.py keygen --test --split 3,5 root
# ./sbl.py keygen --test --split 3,5 l1
# ./sbl.py keygen --test --split 3,5 l2

# ./sbl.py verifykey --key root.1:1 --key root.2:2 --key root.3:3 --pub root
# ./sbl.py certgen --key root.1:1 --key root.3:3 --key root.5:5 --pub l1 l1.cert
# ./sbl.py certgen --key l1.1:1 --key l1.3:3 --key l1.4:4 --pub l2 --chain l1.cert l2.cert
# ./sbl.py sign --key l2.2:2 --key l2.3:3 --key l2.5:5 --code app.bin --cert l2.cert app.signed.bin
# ./sbl.py sign --key l2.1:1 --key l2.2:2 --key l2.5:5 --code app.hex --cert l2.cert app.signed.hex
#
# ./sbl.py verify --root root app.signed.bin
# ./sbl.py verify --root root app.signed.hex
#
# dd if=app.signed.bin of=app.sig bs=1 seek=0 count=512 status=none
# arm-none-eabi-objcopy --update-section .signature=app.sig app.elf app.signed.elf
# arm-none-eabi-objcopy -O binary --gap-fill 0x00 app.signed.elf app.signed.bin
# arm-none-eabi-objcopy -O ihex --gap-fill 0x00 app.signed.elf app.signed2.hex

import argparse
import os
import sys
import struct
import getpass
import hashlib
import time
from datetime import datetime as dt
from datetime import timezone as tz

# python wrapper for libsodium
import nacl.signing
import nacl.encoding
import nacl.secret
import nacl.utils
import nacl.pwhash
import nacl.exceptions

import zbase32
import gf2
import sss
import ihex

SIG_BLOCK_SIZE = 512
SIG_SIZE = 64
CERT_SIZE = 104  # Needs to match src-uc/signature.h struct def
MAX_WHAT_SIZE = 163  # Needs to match src-uc/signature.h APP_MAX_WHAT
HASH_SIZE = 64
PK_SIZE = 32
SK_SIZE = 32

CODE_TYPE_DEC = {b"\x00": "unknown/efi", b"\x01": "mfi", b"\x02": "afi"}
CODE_TYPE_ENC = {" EFI\x00": b"\x00", " MFI\x00": b"\x01", " AFI\x00": b"\x02"}

# As signing keys are 256 bits (32 bytes)
# We are using gf(2,256) for sss splitting
gf256 = gf2.GF2([256, 10, 5, 2, 0])


def rundir():
    return os.path.abspath(os.path.dirname(sys.argv[0]))


SBL_DIR = None


def config_sbl_dir(args=None):
    # See if we already have found .sbl directory location
    global SBL_DIR
    if SBL_DIR is not None:
        return SBL_DIR

    if args and args.sbl:
        SBL_DIR = args.sbl
        return SBL_DIR

    # Search from rundir looking for a .sbl directory
    d = rundir()
    ud = os.path.expanduser("~")
    while d.startswith(ud):
        # checks that sbl directory for storing keys exists and has reasonable
        # permissions
        p = os.path.join(d, ".sbl")
        if os.path.exists(p) and os.path.isdir(p):
            m = os.stat(p).st_mode
            if m & 0o777 == 0o700:
                SBL_DIR = p
                return SBL_DIR
        d = os.path.dirname(d)
    return None


def ensure_sbl_dir():
    if config_sbl_dir() is None:
        print(
            ".sbl does not exist or is not a directory or has permissions other than rwx------"
        )
        exit(1)


def roll():
    while True:
        v = os.urandom(1)[0]
        if v < 42 * 6:  # 42*6 = 252 which is less than 256
            return str((v % 6) + 1)


def passphrase():
    # Generates a Diceware like passphrase with:
    #    8 * log2(6^4) ~= 82 bits of entropy
    #  https://theworld.com/~reinhold/diceware.html
    #  https://www.eff.org/dice
    #
    # Word list is from:
    #  https://www.eff.org/files/2016/09/08/eff_short_wordlist_2_0.txt
    # License is
    #  https://creativecommons.org/licenses/by/3.0/us/
    wl = {}
    with open(f"{rundir()}/eff_short_wordlist_2_0.txt", "rt") as f:
        for line in f.readlines():
            r, w = line.strip().split("\t")
            wl[r] = w
    pp = []
    for _ in range(8):
        r = "".join([roll() for _ in range(4)])
        pp.append(wl[r])
    return "-".join(pp)


# TODO find a better way to allow selecting paramters from command line.
# We probably want "root" to use sensitive or max - rarely done
# We are probably ok with l1/l2 to use interactive.
# We probably want "testing" to use min - not woried about security
# This likely means storing the used paramters with the key to make ux simple.
def pp_config(x=None):
    kdf = nacl.pwhash.scrypt.kdf
    saltn = nacl.pwhash.scrypt.SALTBYTES
    ops = nacl.pwhash.scrypt.OPSLIMIT_INTERACTIVE
    mem = nacl.pwhash.scrypt.MEMLIMIT_INTERACTIVE
    # ops = nacl.pwhash.scrypt.OPSLIMIT_SENSITIVE
    # mem = nacl.pwhash.scrypt.OPSLIMIT_SENSITIVE
    if x is None:
        return kdf, saltn, ops, mem
    elif x == "saltn":
        return saltn


def key_from_pp(pp, pwn, salt=None):
    kdf, saltn, ops, mem = pp_config()
    if salt is None:
        salt = nacl.utils.random(saltn)
        return kdf(pwn, pp, salt, opslimit=ops, memlimit=mem), salt
    else:
        return kdf(pwn, pp, salt, opslimit=ops, memlimit=mem)


def encrypt(key, msg, aad):
    return nacl.secret.Aead(key).encrypt(msg, aad)


def decrypt(key, ct, aad):
    try:
        return nacl.secret.Aead(key).decrypt(ct, aad)
    except nacl.exceptions.CryptoError:
        print("error: unable to decrypt key")
        exit(1)


def encode_split(v, pp=None):
    x, y = v
    if pp is None:
        pp = passphrase()
    key, salt = key_from_pp(pp.encode("utf8"), nacl.secret.Aead.KEY_SIZE)
    msg = int.to_bytes(y.v, SK_SIZE, "little")
    aad = int.to_bytes(x.v, 1, "little") + salt + pp.encode("utf8")
    ct = encrypt(key, msg, aad)
    return f"{x.v}:{zbase32.encode(salt + ct)}:{pp}"


def decode_split(v):
    x, saltct, pp = v.split(":")
    x = int(x)
    saltct = zbase32.decode(saltct)
    saltn = pp_config("saltn")
    salt, ct = saltct[:saltn], saltct[saltn:]
    key = key_from_pp(pp.encode("utf8"), nacl.secret.Aead.KEY_SIZE, salt)
    aad = int.to_bytes(x, 1, "little") + salt + pp.encode("utf8")
    msg = decrypt(key, ct, aad)
    return gf256(x), gf256(int.from_bytes(msg, "little"))


def split_key(key, k, n, test):
    sk = key.encode()
    sk = sss.split(gf256(int.from_bytes(sk, "little")), k, n)
    if test:
        return [encode_split(v, str(idx + 1)) for idx, v in enumerate(sk)]
    else:
        return [encode_split(v) for v in sk]


def join_key(splits):
    sk = [decode_split(v) for v in splits]
    sk = int.to_bytes(sss.join(sk).v, SK_SIZE, "little")
    return nacl.signing.SigningKey(sk)


def prompt_splits():
    splits = []
    prompt = "first"
    while True:
        s = input(f"{prompt} key: ")
        if s == "":
            break
        idx, _ = s.split(":", 1)
        pp = getpass.getpass(f"passphase {idx}: ")
        splits.append(":".join([s, pp]))
        prompt = "next"
    return splits


def dump_splits(name, pk, splits, export_prefix):
    if name:
        for s in splits:
            try:
                x, s, pp = s.split(":")
            except Exception:
                print("error: internal error")
                exit(1)
            try:
                ensure_sbl_dir()
                with open(os.path.expanduser(f"{SBL_DIR}/{name}.{x}"), "wt") as f:
                    f.write(":".join([x, s]))
            except Exception:
                print(f"error: unable to save key split {name}.{x}")
                exit(1)
            print(f"{x}: ", pp)
        try:
            ensure_sbl_dir()
            with open(os.path.expanduser(f"{SBL_DIR}/{name}.pub"), "wt") as f:
                f.write(pk.hex())
        except Exception:
            print(f"error: unable to save public key {name}")
            exit(1)
    elif export_prefix is None:
        for s in splits:
            try:
                x, s, pp = s.split(":")
            except Exception:
                print("error: internal error")
                exit(1)
            print(f"split {x}")
            print(f"  key: {x}:{s}")
            print(f"  pass phrase: {pp}")
        print("public key:")
        print(f"  {pk.hex()}")
    else:
        for s in splits:
            try:
                x, s, pp = s.split(":")
            except Exception:
                print("error: internal error")
                exit(1)
            print(f"export {export_prefix}_{x}={x}:{s}:{pp}")
        print(f"export {export_prefix}_PUB={pk.hex()}")


def load_split_file(p, pp=None):
    if os.path.isfile(p):
        with open(p, "rt") as f:
            x_split = f.read().strip().split(":")
            if len(x_split) == 2:
                x, split = x_split
            elif len(x_split) == 3:
                x, split, pp = x_split
            if pp is None:
                pp = getpass.getpass(f"passphrase {x}: ")
            return ":".join([x, split, pp])
    else:
        print(f"unable to find key file {p}")
        exit(1)


def load_split(key):
    try:
        pp = None
        key_pp = key.split(":")
        if len(key_pp) > 3:
            print(f"error: unable load key split {key}")
            exit(1)
        elif len(key_pp) == 3:
            # full x:y:pp provided
            split = key
        elif len(key_pp) == 2:
            try:
                # x:y provided - need to prompt for pass phrase
                x = int(key_pp[0])
                _ = zbase32.decode(key_pp[1])
                pp = getpass.getpass(f"passphrase {x}: ")
                split = ":".join([key, pp])
            except Exception:
                # file:pp provided or x:y provied with no pp
                ensure_sbl_dir()
                p = os.path.expanduser(f"{SBL_DIR}/{key_pp[0]}")
                split = load_split_file(p, key_pp[1])
        else:
            # file provided
            split = load_split_file(key)
    except Exception as e:
        print(f"error: unable load key split {key}")
        exit(1)
    return split


def load_splits(key):
    if key is None:
        splits = prompt_splits()
    else:
        splits = [load_split(x) for x in key]
    return splits


def decode_what(data):
    # String must be ascii printable (0x20-0x7e inclusive) and null terminated
    printable = bytes(range(0x20, 0x7F))
    if data[-1:] != b"\x00":
        raise ValueError("missing null terminatorin what string")
    if any(b not in printable for b in data[:-1]):
        raise ValueError("bad character in what string")
    return data.decode("ascii")


def what(data):
    # Extract the first NUL terminated "what" string in the data
    try:
        data = data[data.index(b"@(#)") + 4 :]
        return decode_what(data[: data.index(b"\x00") + 1])
    except ValueError:
        print("error: bad/missing what string")
        exit(1)


def keygen(args):
    # Generate a new random signing key, split it and then save
    sk = nacl.signing.SigningKey.generate()
    pk = sk.verify_key.encode()
    splits = split_key(sk, *args.split, args.test)
    dump_splits(args.file, pk, splits, args.export_prefix)


def resplit(args):
    # Load key
    splits = load_splits(args.key)
    key = join_key(splits)
    pk = key.public_key.encode()
    print(f"resplitting key: {pk.hex()}")

    # Split it and then save
    splits = split_key(key, *args.split)
    dump_splits(args.file, pk, splits, args.export_prefix)


def load_cert(cert):
    try:
        with open(cert, "rt") as f:
            cert = f.read()
        cert = bytes.fromhex(cert)
        if len(cert) != CERT_SIZE and len(cert) != CERT_SIZE * 2:
            raise Exception("invalid cert")
    except Exception:
        print(f"error: unable to load cert {cert}")
        exit(1)
    return cert


def load_pub(pub):
    pk = None
    try:
        if SBL_DIR:
            p = os.path.expanduser(f"{SBL_DIR}/{pub}.pub")
            if os.path.isfile(p):
                with open(p, "rt") as f:
                    pk = f.read()
        if pk is None:
            # Assume the key is passed as hex on command line.
            pk = pub
        pk = bytes.fromhex(pk)
    except Exception:
        print(f"error: unable to load public key {pub}")
        exit(1)
    return nacl.signing.VerifyKey(pk)


def get_date(date_str):
    if date_str:
        try:
            sig_date = int(date_str)
        except ValueError:
            try:
                sig_date = dt.strptime(date_str, "%Y-%m-%dT%H:%M:%S%z").timestamp()
            except ValueError:
                print(f"error: unable to parse {date_str} as a date")
                exit(1)
    else:
        sig_date = int(time.time())
    return sig_date


def certsign(args):
    cert_pk = load_pub(args.pub)
    cert_date = get_date(args.date)

    cert = struct.pack("<Q", cert_date) + cert_pk.encode()

    # Add chaining info
    if args.chain:
        if os.path.isfile(args.chain):
            with open(args.chain, "rt") as f:
                chain = f.read()
        else:
            chain = args.chain
        chain = bytes.fromhex(chain)
        cert = cert + chain

    # Load key
    splits = load_splits(args.key)
    signing_key = join_key(splits)

    # Sign and verify
    sig = signing_key.sign(cert).signature
    try:
        verify_key = signing_key.verify_key
        verify_key.verify(cert, sig)
    except ValueError:
        print("error: unable to validate signature")
        exit(1)

    # Save output
    cert = sig + cert
    if args.file:
        with open(args.file, "wt") as f:
            f.write(cert.hex())
    else:
        print(f"cert: {cert.hex()}")


def load_code(file):
    code_ext = os.path.splitext(file)[1].lower()
    try:
        if code_ext == ".bin":
            with open(file, "rb") as f:
                code = f.read()
            return None, None, code
        elif code_ext == ".hex":
            with open(file, "rt") as f:
                ss, image = ihex.load(f)
            if len(image) != 1:
                print("error: hex file has gaps or is empty")
                exit(1)
            addr, code = image[0]
            return ss, addr, code
        else:
            print("error: only support .bin and .hex files for code")
            exit(1)
    except Exception:
        print(f"error: unable to load code from {file}")
        exit(1)


def save_code(file, ss, addr, code):
    try:
        ext = os.path.splitext(file)[1].lower()
        if ext == ".bin":
            with open(file, "wb") as f:
                f.write(code)
        elif ext == ".hex":
            if addr is None or ss is None:
                print("can only save hex output if code input is also hex")
                exit(1)
            with open(file, "wt") as f:
                ihex.dump(f, ss, [(addr, code)])
            pass
        else:
            print(f'unknown file extension "{ext}"')
            exit(1)
    except Exception:
        print(f"error: writing to {file}")
        exit(1)


def split_code(code):
    # get hash and version info
    sig, code = code[:SIG_BLOCK_SIZE], code[SIG_BLOCK_SIZE:]
    code_what = what(code)
    if len(code_what) > MAX_WHAT_SIZE:
        print(f'error: code version string too long: "{code_what}"')
        exit(1)
    sha512 = hashlib.sha512()
    sha512.update(code)
    code_hash = sha512.digest()
    code_n = len(code)
    return sig, code_hash, code_n, code_what, code


def sign(args):
    ss, addr, code = load_code(args.code)
    _, code_hash, code_n, code_what, code = split_code(code)
    cert = load_cert(args.cert)
    sig_date = get_date(args.date)

    if code_what[-5:] not in CODE_TYPE_ENC:
        print("error: invalid code type")
        exit(1)
    code_type = CODE_TYPE_ENC[code_what[-5:]]

    # Generate the data to be signed
    sigdata = (
        struct.pack("<IQ", code_n, sig_date)
        + code_hash
        + code_type
        + code_what.encode("ascii")
    )
    pad = SIG_BLOCK_SIZE - len(sigdata) - len(cert) - SIG_SIZE
    if pad < 0:
        print("error: internal error")
        exit(1)
    sigdata = sigdata + b"\xff" * pad + cert
    if len(sigdata) != SIG_BLOCK_SIZE - SIG_SIZE:
        print("error: internal error")
        exit(1)

    # Load key
    splits = load_splits(args.key)
    signing_key = join_key(splits)

    # Sign
    sig = signing_key.sign(sigdata).signature

    try:
        pk = signing_key.verify_key.encode()
        verify_key = nacl.signing.VerifyKey(pk)
        verify_key.verify(sigdata, sig)
    except nacl.exceptions.BadSignatureError:
        print("error: unable to validate code signature")
        exit(1)

    # Save output
    if len(sig) + len(sigdata) != SIG_BLOCK_SIZE:
        print("internal error")
        exit(1)
    save_code(args.file, ss, addr, sig + sigdata + code)


def cert_present(cert):
    return not all([x == 255 for x in cert])


def verify_cert(name, cert, pk, date, args):
    sig, sig_block = cert[:64], cert[64:]
    try:
        pk.verify(sig_block, sig)
    except nacl.exceptions.BadSignatureError:
        print("error: unable to validate cert signature")
        exit(1)

    (cert_date,), new_pk = (
        struct.unpack("<Q", sig_block[:8]),
        sig_block[8 : 8 + PK_SIZE],
    )
    if cert_date < date:
        print("error: date in cert earlier than signers date")
        exit(1)
    if len(new_pk) != PK_SIZE:
        print("error: internal error")
        exit(1)
    new_pk = nacl.signing.VerifyKey(new_pk)

    if args.debug:
        print(f"info: {name} cert valid")
        print(f"  pk:       {new_pk.encode().hex()}")
        print(
            f'  date:     {dt.fromtimestamp(cert_date, tz=tz.utc).strftime("%Y-%m-%dT%H:%M:%SZ")}'
        )

    return new_pk, cert_date


def verify_key(args):
    pk = load_pub(args.pub)
    splits = load_splits(args.key)
    sk = join_key(splits)
    if pk.encode().hex() == sk.verify_key.encode().hex():
        print("key valid")
    else:
        print("key invalid:")
        print(f"pk from file:   {pk.encode().hex()}")
        print(f"pk from splits: {sk.verify_key.encode().hex()}")


def verify(args):
    _, _, code = load_code(args.code)
    sig_block, code_hash, code_n, code_what, code = split_code(code)

    pk = load_pub(args.root)
    pk_date = 0

    if args.debug:
        print("info: root")
        print(f"  pk:       {pk.encode().hex()}")
        print(
            f'  date:     {dt.fromtimestamp(pk_date, tz=tz.utc).strftime("%Y-%m-%dT%H:%M:%SZ")}'
        )

    sig, sig_block = sig_block[:SIG_SIZE], sig_block[SIG_SIZE:]
    certs = sig_block[-CERT_SIZE * 2 :]
    cert2, cert1 = certs, certs[CERT_SIZE:]

    pk, pk_date = verify_cert("primary", cert1, pk, pk_date, args)
    pk, pk_date = verify_cert("secondary", cert2, pk, pk_date, args)
    try:
        pk.verify(sig_block, sig)
    except nacl.exceptions.BadSignatureError:
        print("error: unable to validate code signature")
        exit(1)

    sig_block = sig_block[: -CERT_SIZE * 2]
    sig_block, sig_what = sig_block[:-MAX_WHAT_SIZE], sig_block[-MAX_WHAT_SIZE:]
    sig_block, code_type = sig_block[:-1], sig_block[-1:]
    sig_block, code_hash2 = sig_block[:-HASH_SIZE], sig_block[-HASH_SIZE:]
    code_n2, date = struct.unpack("<IQ", sig_block)
    if code_n != code_n2:
        print("error: code length mismatch")
        exit(1)
    try:
        sig_what = decode_what(sig_what[: sig_what.index(b"\x00") + 1])
        if sig_what != code_what:
            raise ValueError("signature what and code what don't match")
        sig_what = sig_what[:-1]  # remove null terminator
    except ValueError:
        print(f"error: invalid what string {sig_what}")
        exit(1)
    if code_hash != code_hash2:
        print("error: code hash mismatch")
        exit(1)
    if pk_date > date:
        print("error: cert dates later than signature date")
        exit(1)
    if code_type not in CODE_TYPE_DEC:
        print("error: invalid code type")
        exit(1)

    if args.debug:
        print("info: image valid")
        print(f"  build-id: {sig_what}")
        print(f"  type:     {CODE_TYPE_DEC[code_type]}")
        print(f"  length:   {code_n}")
        print(f"  hash:     {code_hash.hex()}")
        print(
            f'  date:     {dt.fromtimestamp(date, tz=tz.utc).strftime("%Y-%m-%dT%H:%M:%SZ")}'
        )
    else:
        print("image valid")


def config_bl(args):
    # ENHANCEMENT: This checking is very nRF52 centric needs to accomodate
    # other processor configurations
    try:
        manu_data_size = int(args.manu_data_size)
        if manu_data_size <= 0:
            raise ValueError("manu-data-size must be > 0")
        if manu_data_size % 4096 != 0:
            raise ValueError("manu-data-size must multiple 4096")
    except ValueError as e:
        print(f"error: invalid manu-data-size: {e}")
        exit(1)

    try:
        max_app_size = int(args.max_app_size)
        if max_app_size <= 0:
            raise ValueError("max-app-size must be > 0")
        if max_app_size % 4096 != 0:
            raise ValueError("max-app-size must multiple 4096")
    except ValueError as e:
        print(f"error: invalid max-app-size: {e}")
        exit(1)

    if 32768 + 8192 + manu_data_size + max_app_size * 2 + 4096 > 1024 * 1024:
        print("error: invalid config for part: out of FLASH")
        exit(1)

    # This needs to match src-uc/apputils.c initalization for ROOT_CODE_PK
    def_pk = bytes.fromhex(
        "73bed90ce4a9505ff8235e51fece9d4ddeb0fcd44c48e422f200c6b78bd481bf"
    )
    pk = load_pub(args.root).encode()

    # The chance of collision is vanishingly small - but we check in case
    # it "magically" occures.
    if pk == def_pk:
        print("error: invalid root key - same as default PK")
        exit(1)
    if len(pk) != len(def_pk):
        print("error: invalid root key - bad size")
        exit(1)
    ss, addr, code = load_code(args.code)
    if args.verify:
        try:
            idx = code.index(pk)
            idx = idx + len(pk)
            config = struct.unpack("<" + "I" * 8, code[idx : idx + 8 * 4])
            config = {
                "bl-len": config[0],
                "bl-state": config[1],
                "bl-state-len": config[2],
                "manu-data": config[3],
                "manu-data-len": config[4],
                "slot0": config[5],
                "slot1": config[6],
                "slot-len": config[7],
            }

            print("sbl configured with:")
            print(f"  pk:             {pk.hex()}")
            print(f"  bl-len:         {config['bl-len']}")
            print(f"  bl-state:       0x{config['bl-state']:08x}")
            print(f"  bl-state-len:   {config['bl-state-len']}")
            print(f"  manu-data:      0x{config['manu-data']:08x}")
            print(f"  manu-data-len:  {config['manu-data-len']}")
            print(f"  slot0:          0x{config['slot0']:08x}")
            print(f"  slot1:          0x{config['slot1']:08x}")
            print(f"  slot-len:       {config['slot-len']}")
        except Exception:
            print(f"error: SBL image not configured with {args.root} PK")
            exit(1)
    else:
        try:
            idx = code.index(def_pk)
            if any(x != 0 for x in code[idx + 32 : idx + 64]):
                raise Exception("mem config not all zeros")

            # Default nRf52 config
            config = {
                "bl-len": 32768,
                "bl-state": 32768,
                "bl-state-len": 8192,
                "manu-data": 32768 + 8192,
                "manu-data-len": manu_data_size,
                "slot0": 32768 + 8192 + manu_data_size,
                "slot1": 32768 + 8192 + manu_data_size + max_app_size,
                "slot-len": max_app_size,
            }
            config = pk + struct.pack(
                "<IIIIIIII",
                config["bl-len"],
                config["bl-state"],
                config["bl-state-len"],
                config["manu-data"],
                config["manu-data-len"],
                config["slot0"],
                config["slot1"],
                config["slot-len"],
            )
            code2 = code[:idx] + config + code[idx + len(config) :]
        except Exception:
            print(
                "error: invalid SBL image - missing default PK or config not 0in image"
            )
            exit(1)
        try:
            _ = code2.index(def_pk)
            print("error: invalid SBL image - more than 1 default PK in image")
            exit(1)
        except Exception:
            save_code(args.code, ss, addr, code2)


def split(string):
    # takes a string of the form k,n and returns a list ints [k, n]
    # otherwise raises an argument error
    try:
        k, n = [int(v) for v in string.split(",")]
    except Exception:
        raise argparse.ArgumentTypeError(
            f"invalid split({string}) - needs to be in form [K],[N]"
        )
    if k <= 0 or n <= 0 or k > n:
        raise argparse.ArgumentTypeError(f"invalid k({k}) or n({n})")
    return [k, n]


if __name__ == "__main__":
    parser = argparse.ArgumentParser("sbl")
    parser.add_argument("--debug", action="store_true", help="debug output")
    parser.add_argument("--sbl", help=".sbl directory to use")

    sub = parser.add_subparsers(required=True, dest="cmd")
    sp = sub.add_parser("keygen", help="generates a new key")
    sp.add_argument(
        "-t",
        "--test",
        action="store_true",
        help="make passephrase for split equal to split index",
    )
    sp.add_argument(
        "-p",
        "--export-prefix",
        help="if file is empty use sh export format with prefix",
    )
    sp.add_argument(
        "-s",
        "--split",
        type=split,
        required=True,
        help="[K],[N] - split key into [N] splits with quarum of [K]",
    )
    sp.add_argument(
        "file",
        nargs="?",
        help="if provided, outputs to [FILE].1[, [FILE].2, ...] and [FILE].pub which are stored in the .sbl directory",
    )

    sp = sub.add_parser("resplit", help="resplits a key into a new set of splits")
    sp.add_argument(
        "-s",
        "--split",
        type=split,
        required=True,
        help="[K],[N] - re-split key into [N] splits with quarum of [K]",
    )
    sp.add_argument(
        "-k",
        "--key",
        action="append",
        help="key to be resplit - should be repeated [K] times where [K] is the quarum of the current key",
    )
    sp.add_argument(
        "file",
        nargs="?",
        help="outputs to [FILE][.1, [FILE].2, ...] and [FILE].pubwhich by default are stored in the ~/.sbl directory",
    )

    sp = sub.add_parser("certgen", help="generates a new certificate and signs it")
    sp.add_argument(
        "-k",
        "--key",
        action="append",
        help="key to use for signing certificate - should be repeated [K] times where [K] is the quarum of the key.",
    )
    sp.add_argument("-d", "--date", help="use [DATE] instead of current time")
    sp.add_argument(
        "-p",
        "--pub",
        required=True,
        help="the public key the cert is being created for",
    )
    sp.add_argument(
        "-c",
        "--chain",
        help="the parent certificate in chain - usually the cert for [KEY]",
    )
    sp.add_argument("file", nargs="?", help="output cert to [FILE]")

    sp = sub.add_parser("sign", help="sign a file")
    sp.add_argument(
        "-k",
        "--key",
        action="append",
        help="key to use for signing certificate - should be repeated [K] times where [K] is the quarum of the key.",
    )
    sp.add_argument("-d", "--date", help="use [DATE] instead of current time")
    sp.add_argument("--code", required=True, help="the binary file to sign")
    sp.add_argument(
        "--cert",
        required=True,
        help="the certificate chain for [KEY] to include in the signature",
    )
    sp.add_argument("file", help="output signature to [FILE]")

    sp = sub.add_parser("verify", help="verify a signature in binary code image")
    sp.add_argument(
        "-r", "--root", required=True, help="root public key used for verification"
    )
    sp.add_argument("code", help="the binary code file to verify")

    sp = sub.add_parser("verifykey", help="verify key pk/sk")
    sp.add_argument(
        "-k",
        "--key",
        action="append",
        help="key to verify - should be repeated [K] times where [K] is the quarum of the key.",
    )
    sp.add_argument(
        "pub",
        help="the public key the cert is being created for",
    )

    sp = sub.add_parser("config", help="configure SBL image")
    sp.add_argument(
        "-r",
        "--root",
        required=True,
        help="root public key to configure SBL image with",
    )
    sp.add_argument(
        "--manu-data-size",
        default="4096",
        help="manufacturing area data size",
    )
    sp.add_argument(
        "--max-app-size",
        default="491520",
        help="maximum size of an application",
    )
    sp.add_argument(
        "-v",
        "--verify",
        action="store_true",
        help="verify that SBL is configured with root",
    )
    sp.add_argument("code", help="SBL binary to configure")

    args = parser.parse_args()
    config_sbl_dir(args)

    if args.cmd == "keygen":
        keygen(args)
    elif args.cmd == "resplit":
        resplit(args)
    elif args.cmd == "certgen":
        certsign(args)
    elif args.cmd == "sign":
        sign(args)
    elif args.cmd == "verify":
        verify(args)
    elif args.cmd == "verifykey":
        verify_key(args)
    elif args.cmd == "config":
        config_bl(args)
    else:
        print(f"error: unknown cmd: {args.cmd}")
        exit(1)
