"""Binary image -> $readmemh file, one 32-bit little-endian word per line,
padded with zeros to the RAM size."""
import sys

src, dst, words = sys.argv[1], sys.argv[2], int(sys.argv[3])
data = open(src, "rb").read()
if len(data) > 4 * words:
    sys.exit(f"bin2hex: image is {len(data)} bytes, RAM holds {4 * words}")
data += bytes(4 * words - len(data))
with open(dst, "w") as f:
    for i in range(0, len(data), 4):
        f.write(f"{int.from_bytes(data[i:i + 4], 'little'):08x}\n")
