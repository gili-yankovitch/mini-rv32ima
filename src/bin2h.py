#!/usr/bin/python3

import argparse
template = """#include <stdint.h>
#ifndef __{INCLUDEGUARD}__
#define __{INCLUDEGUARD}__

uint8_t _{DATA_VAR}_data [] = {DATA};

#endif __{INCLUDEGUARD}__
"""

def main(binfile, outfile):
    with open(binfile, "rb") as f:
        data = "\"" + "".join([ "\\" + "x" + hex(b)[2:] for b in f.read() ]) + "\""

    with open(outfile, "w") as f:
        f.write(template.format(INCLUDEGUARD = outfile.upper().replace(".", "_"),
            DATA_VAR = outfile.lower().replace(".", "_"),
            DATA = data))

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("binfile")
    parser.add_argument("output")
    args = parser.parse_args()
    main(args.binfile, args.output)
