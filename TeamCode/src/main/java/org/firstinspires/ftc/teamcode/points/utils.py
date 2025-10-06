#!/usr/bin/env python3
import base64
import json
import os
from cryptography.fernet import Fernet
from cryptography.hazmat.primitives import hashes
from cryptography.hazmat.primitives.kdf.pbkdf2 import PBKDF2HMAC
from getpass import getpass
from typing import Tuple

FILE = "points.cpdb"
HEADER = b"CPDBv1"
SALT_SIZE = 16
ITER = 390_000

def derive_key(password: bytes, salt: bytes) -> bytes:
    kdf = PBKDF2HMAC(algorithm=hashes.SHA256(), length=32, salt=salt, iterations=ITER)
    return base64.urlsafe_b64encode(kdf.derive(password))

def read_db(password: bytes) -> dict:
    if not os.path.exists(FILE):
        raise FileNotFoundError(FILE)
    with open(FILE, "rb") as f:
        raw = f.read()
    if not raw.startswith(HEADER):
        raise ValueError("Bad file header")
    salt = raw[len(HEADER):len(HEADER)+SALT_SIZE]
    ciphertext = raw[len(HEADER)+SALT_SIZE:]
    key = derive_key(password, salt)
    f = Fernet(key)
    data = f.decrypt(ciphertext)
    return json.loads(data.decode()), salt

def write_db(obj: dict, password: bytes, salt: bytes) -> None:
    key = derive_key(password, salt)
    f = Fernet(key)
    ciphertext = f.encrypt(json.dumps(obj, separators=(",", ":")).encode())
    with open(FILE, "wb") as fh:
        fh.write(HEADER + salt + ciphertext)
