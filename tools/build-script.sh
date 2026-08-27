# Step 1: patch the header
source ./venv/bin/activate
python3 -c "
path = './build/flutter/build/linux/x64/release/python/include/python3.12/pyconfig.h'
with open(path) as f:
    content = f.read()
content = content.replace(
    '#define _POSIX_C_SOURCE 200809L',
    '#ifndef _POSIX_C_SOURCE\n#define _POSIX_C_SOURCE 200809L\n#endif'
).replace(
    '#define _XOPEN_SOURCE 700',
    '#ifndef _XOPEN_SOURCE\n#define _XOPEN_SOURCE 700\n#endif'
)
with open(path, 'w') as f:
    f.write(content)
print('Patched!')
"

# Step 2: build again (don't clean, so the patched header is reused)
flet build linux