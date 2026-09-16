import argparse
import olefile


def records(path):
    ole = olefile.OleFileIO(path)
    data = ole.openstream("FileHeader").read()
    pos = 0
    while pos + 4 <= len(data):
        size = int.from_bytes(data[pos:pos + 4], "little")
        pos += 4
        if size <= 0 or pos + size > len(data):
            break
        raw = data[pos:pos + size].rstrip(b"\0")
        pos += size
        text = raw.decode("windows-1252", errors="replace")
        props = {}
        for field in text.split("|"):
            if "=" in field:
                key, value = field.split("=", 1)
                props[key] = value
        yield props, text


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("path")
    parser.add_argument("--match", default="")
    args = parser.parse_args()
    needle = args.match.upper()
    for index, (props, text) in enumerate(records(args.path)):
        if not needle or needle in text.upper():
            print(index, text)


if __name__ == "__main__":
    main()
