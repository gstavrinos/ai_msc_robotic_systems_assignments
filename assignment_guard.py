import sys
import base64
from os.path import exists
from cryptography.fernet import Fernet
from cryptography.hazmat.primitives import hashes
from cryptography.hazmat.primitives.kdf.pbkdf2 import PBKDF2HMAC

if len(sys.argv) == 3 and \
        (sys.argv[1].endswith(".enc") or sys.argv[1].endswith(".dec")):
    key = sys.argv[2].encode("utf-8")
    kdf = PBKDF2HMAC(
        algorithm=hashes.SHA256(),
        length=32,
        salt=b"sigouro0",
        iterations=480000,
    )
    base64_key = base64.urlsafe_b64encode(kdf.derive(key))

    fernet = Fernet(base64_key)

    input_file = None

    if exists(sys.argv[1]):
        with open(sys.argv[1], "rb") as file:
            input_file = file.read()
    else:
        print("Input file does not exist.")
        exit(1)

    output_name = ""
    if sys.argv[1].endswith(".enc"):
        output_name = sys.argv[1].replace(".enc", ".dec")
        output_file = fernet.decrypt(input_file)
    elif sys.argv[1].endswith(".dec"):
        output_name = sys.argv[1].replace(".dec", ".enc")
        output_file = fernet.encrypt(input_file)

    with open(output_name, "wb") as of:
        of.write(output_file)
else:
    print("The script needs two arguments.\nThe first one should be the file to be encrypted/decrypted.\n(ending with .dec/.enc)\nThe second one should be the password.")
    exit(2)
