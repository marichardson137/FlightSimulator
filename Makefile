OSX_COMPILER = clang
WIN_COMPILER = gcc
WEB_COMPILER = emcc
C_STD = -std=c99
SOURCE_LIBS = -Ilib/
CFILES = src/*.c

# MacOS specific flags and libraries
OSX_OPT =  -g -Wall -Llib/ -framework CoreVideo -framework IOKit -framework Cocoa -framework GLUT -framework OpenGL lib/libraylib_osx.a
OSX_OUT = -o "bin/build_osx"

build_osx:
	$(OSX_COMPILER) $(C_STD) $(CFILES) $(SOURCE_LIBS) $(OSX_OUT) $(OSX_OPT)

# Windows specific flags and libraries
WIN_OPT = -g -Wall -Llib/ lib/libraylib_win.a -lopengl32 -lgdi32 -lwinmm
WIN_OUT = -o "bin/build_win.exe"

build_win:
	$(WIN_COMPILER) $(C_STD) $(CFILES) $(SOURCE_LIBS) $(WIN_OUT) $(WIN_OPT)

# Web specific flags and libraries
WEB_OPT = -Llib/ lib/libraylib_web.a -Wall -D_DEFAULT_SOURCE -Wno-missing-braces -Wunused-result -Os -s USE_GLFW=3 -s ASYNCIFY -s TOTAL_MEMORY=67108864 -s FORCE_FILESYSTEM=1 --shell-file src/shell.html -DPLATFORM_WEB -s 'EXPORTED_FUNCTIONS=["_free","_malloc","_main"]' -s EXPORTED_RUNTIME_METHODS=ccall
WEB_OUT = -o "index.html"

build_web:
	$(WEB_COMPILER) $(C_STD) $(CFILES) $(SOURCE_LIBS) $(WEB_OUT) $(WEB_OPT)

run:
	make build_osx
	./bin/build_osx
	