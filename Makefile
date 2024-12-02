SRCS:=mini-rv32ima.c decoder.c mmio.c spi.c w25q.c

all: mini-rv32ima wasm

mini-rv32ima: ${SRCS}
	gcc -o $@ -O2 -ffunction-sections -fdata-sections -Wl,--gc-sections -s ${SRCS}

wasm: ${SRCS}
	emcc -DWASM -o mini-rv32ima.js -s ASYNCIFY=1 -s EXPORTED_RUNTIME_METHODS=ccall -s EXPORTED_FUNCTIONS='["_main", "_w25q_cmd"]' ${SRCS}

# For converting the .dtb into a .h file for embeddding.
bintoh:
	echo "#include <stdio.h>" > bintoh.c
	echo "int main(int argc,char ** argv) {if(argc==1) return -1; int c, p=0; printf( \"static const unsigned char %s[] = {\", argv[1] ); while( ( c = getchar() ) != EOF ) printf( \"0x%02x,%c\", c, (((p++)&15)==15)?10:' '); printf( \"};\" ); return 0; }" >> bintoh.c
	gcc bintoh.c -o bintoh

clean:
	rm -rf mini-rv32ima mini-rv32ima.js mini-rv32ima.wasm

