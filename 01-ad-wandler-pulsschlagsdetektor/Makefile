CFLAGS_DEBUG = -DCOMPILER_EXPLORER -Wall -Wextra -fsanitize=address -fno-omit-frame-pointer -std=c99
CFLAGS_WATCH = -DCOMPILER_EXPLORER -std=c99

TARGET = ./tmp/out
SOURCE = main.c

clean:
	rm -rf ./tmp 

makeOutputDir: 
	mkdir -p tmp

compileWatch: 
	gcc $(SOURCE) -o ./tmp/out $(CFLAGS_RUN)

compileDebug: 
	gcc $(SOURCE) -o ./tmp/out $(CFLAGS_DEBUG)

run: 
	./tmp/out

debug:
	make compileDebug run

watch:
	echo $(SOURCE) | entr -c make clean makeOutputDir compileWatch run


.PHONY: clean makeOutputDir compileWatch compileDebug run debug watch 
