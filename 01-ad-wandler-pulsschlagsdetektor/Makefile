#-----------------------------------------------------------------------------
#ToDo
# - Trace32 
#   - snoop 
#     - im RAM-Modus
#     - I2C Analyser nutzen / Testen
#-----------------------------------------------------------------------------
# Globale Einstellungen

#Pfad + Prefix zum GCC Toolchain 
#GNU_ARM  := ../devkitARM/bin/arm-none-eabi-
#GNU_ARM  := arm-none-eabi-
#GNU_ARM   := /lib/gcc/gcc-arm-none-eabi-10.3-2021.10/bin/arm-none-eabi-
GNU_ARM   := /lib/gcc/arm-gnu-toolchain-14.3.rel1-x86_64-arm-none-eabi/bin/arm-none-eabi-

TRACE32_INSTALL_DIR := /mnt/c/Programme.Portable/T32MINI.202009
#TRACE32_INSTALL_DIR := /mnt/c/Programme.Portable/T32MINI.202402

#Einstungen koennten zwar mittels uname automatisch ermittelt werden,
#so ist es aber sicherer
 TRACE32_OS := WINDOWS
#TRACE32_OS := LINUX
#TRACE32_OS := MACOS 

#PROJECT_TARGET = DEBUG / RELEASE
#DEBUG           -> Compilierung fuer max. Debugger Unterstuetzung (ohne Codeoptimierung)
#RELEASE         -> Compilierung fuer min. Debugger Unterstuetzung (mit Codeoptimierung)
 PROJECT_TARGET := DEBUG
#PROJECT_TARGET := RELEASE

#Debugger + Speicheraufteilung (letzteres siehe link.ld)
#MODE := TRACE32_ROM
MODE := TRACE32_RAM
#MODE := TRACE32_SIM
#MODE := GDBOPENOCD_RAM
#MODE := SAMBA_RAM

#Erzeugung der LST-Files Aktivieren
#LST_FILES := NO
LST_FILES := YES

#Kommandozeilenausgabe ausschalten
.SILENT:

#-----------------------------------------------------------------------------
# Prozessor Einstellungen
ARM_CORE := arm7tdmi

#-----------------------------------------------------------------------------
#Projekt/Applikationsname
APP_NAME = demo

#Source-Files
#32-Bit ARM-Mode   --> Dateiendung *.c
#16-Bit THUMB-Mode --> Dateiendung *.ct
APP_SRC_C  = main low_level_init  newlib_syscalls \
		   $(LIB_DIR)/byte_fifo_cb \
		   $(LIB_DIR)/helper       \
           $(LIB_DIR)/aic          \
		   $(LIB_DIR)/systick      \
           $(LIB_DIR)/term         \
		   $(LIB_DIR)/display      \
		   $(LIB_DIR)/nxt_lcd      \
		   $(LIB_DIR)/nxt_spi      \
		   $(LIB_DIR)/nxt_avr      \
		   $(LIB_DIR)/twi          
	   
		   
APP_SRC_ASM  = startup $(LIB_DIR)/isr

#-----------------------------------------------------------------------------
#Statische Libarys
#libc      --> Vollstaendige C-Library (incl. FloatingPoint printf/scanf)
#libc_nano --> Kompaktere Version des Standard-C Library
#              (printf/scanf ohne %efg Unterstuetzung)
#              (Alternativ _printf_float() _scanf_float())
#libg      --> Debugging-enabled C-Library
#libg_nano --> Debugging-enabled kompakte C-Library

ifeq ($(PROJECT_TARGET),DEBUG)
  #SHLIBS   += -lg
  SHLIBS    += -lg_nano
else ifeq ($(PROJECT_TARGET),RELEASE)
  #SHLIBS   += -lc
  SHLIBS    += -lc_nano
endif

#Mathematik Library fuer sin() cos() pow()  sqrt() ...
SHLIBS   += -lm

#Suchpfade fuer Header-Dateien (z.B. der Librarys)
CCINC   = -I$(dir $(GNU_ARM))../include

#-----------------------------------------------------------------------------
# Verzeichnisse

BIN_DIR      := bin
DEP_DIR      := dep
LIB_DIR      := ./lib
TRACE32_INC  := ./trace32
OPENOCD_INC  := ./openOCD
TEMPLATE_INC := ./template

#-----------------------------------------------------------------------------
# Variablen fuer Entwicklungsumgebung setzen

CC        := $(GNU_ARM)gcc
CPP       := $(GNU_ARM)g++
ASM       := $(GNU_ARM)as
LINK      := $(GNU_ARM)gcc
BIN       := $(GNU_ARM)objcopy
NM        := $(GNU_ARM)nm
DUMP      := $(GNU_ARM)objdump --syms
SIZE      := $(GNU_ARM)size
RM        := rm -rf
MKDIR     := mkdir -p
TOUCH     := touch
MV        := mv
SED       := sed    #Alternativ zu sed könnte auch der C Preprozessor genutzt werden
                    #gcc -p $< $@
GREP      := grep
BIN2SAMBA := ../bin2samba/bin2samba.exe


#-----------------------------------------------------------------------------
# Konfiguration pruefen

#Pruefen, ob GNU_ARM Verzeichnis 'existiert'
#- Test auf vorhandensein einer Datei auch über  $(shell command -v datei) möglich
ifneq ($(shell test -e $(GNU_ARM)gcc && echo -n yes),yes)
  $(error Invalid Path GNU_ARM='$(GNU_ARM)')
endif

#Pruefen, ob TRACE32_INSTALL_DIR Verzeichnis existiert
ifneq ($(shell test -e $(TRACE32_INSTALL_DIR) && echo -n yes),yes)
  $(error Invalid Path TRACE32_INSTALL_DIR='$(TRACE32_INSTALL_DIR)')
endif

ifeq ($(TRACE32_OS),WINDOWS)
else ifeq ($(TRACE32_OS),LINUX)
else ifeq ($(TRACE32_OS),MACOS)
else
  $(error Wrong/Unknown Setting for TRACE32_OS='$(TRACE32_OS)')
endif

ifeq ($(PROJECT_TARGET),DEBUG)
else ifeq ($(PROJECT_TARGET),RELEASE)
else
  $(error Wrong/Unknown Setting for PROJECT_TARGET='$(PROJECT_TARGET)')
endif

ifeq ($(MODE),TRACE32_ROM)
else ifeq ($(MODE),TRACE32_RAM)
else ifeq ($(MODE),TRACE32_SIM)
else ifeq ($(MODE),SAMBA_RAM)
else ifeq ($(MODE),GDBOPENOCD_RAM)
else
	$(error Wrong/Unknown Setting for MODE='$(MODE)')
endif

ifeq ($(LST_FILES),YES)
else ifeq ($(LST_FILES),NO)
else
  $(error Wrong/Unknown Setting for LST_FILES='$(LST_FILES)') 
endif

#-----------------------------------------------------------------------------
# Allgemeingueltige Variablen setzen
RED   := \033[0;31m
GREEN := \033[0;32m
NC    := \033[0m

#-----------------------------------------------------------------------------
# Compiler/Linker-Flags
# -O0                Reduce compilation time and make debugging produce the expected results (default)
# -O -O1             The Compiler tries to reduce code size and execution time, without
#                    performing any optimizations that take a great deal of compilation tim
# -O2                Optimize even more. GCC performs nearly all supported optimizations that
#                    do not involve a space-speed tradeoff. As compared to '-O', this option
#                    increases both compilation time and the performance of the generated code
# -O3                Optimize yet more. '-O3' turns on all optimizations specified by '-O2'
#                    and also turns on the ....
# -Os                Optimize for size. '-Os' enables all '-O2' optimizations that do not typically
#                    increase code size. It also performs further optimizations designed to 
#                    reduce code size
# -Ofast             Disregard strict standards compliance. '-Ofast' enables all '-O3'
#                    optimizations. It also enables optimizations that are not valid for all
#                    standard edit-compile-debug cycle, offering a reasonable level of optimization
#                    while maintaining fast compilation and a good debugging experience.
# -g                 Produce debugging information in the operating system s native format (stabs,
#                    COFF, XCOFF, or DWARF 2). GDB can work with this debugging information.
# -Wall              Alle Warnings einschalten
# -fno-builtin       Einbettung von Funktionen deaktivieren
# -fdata-sections    Teilt jeder Variablen eine eigene Section zu
# -ffunction-section Teilt jeder Funktion  eine eigene Section zu (main -> .text.main)
# -Wl,--gc-sections  Der Linker verwirft unreferenzierte Sections und packt diese
#                    nicht ins Binary.
# -fno-rtti          C++: Disable generation of information about every class with virtual functions 
#                    for use by the C++ run-time type identification features (dynamic_cast and typeid)
# -fno-exceptions    C++: Disable the warning about the case when an exception handler is shadowed 
#                    by another handler, which can point out a wrong ordering of exception handlers.
# -flto              Aktivierung der Link Time Optimizatio, so dass Funktionsaufrufe ueber
#                    Dateigrenzen optimiert werden k nnen
#                    Die LTO erkennt die ISR's und den Interrupt Vector
#                    m glicherweise als "unbenutzt" und optimiert sie daher weg. 
#                    Dies kann durch Markierung der Funktionen & Variablen mit
#                    "__attribute__ ((used))" verhindert werden.
# -mcpu=arm7tdmi     Zielarchitektur setzen (Alternativ cortex-m0 corex-m32 ..)
# -mthumb            Zielarchitektur (16-Bit thumb Befehlssatz erzeugen)
# -mfloat-abi=soft   FloatingPointUnterst tztung (soft / hard)
# -mfpu=fpv4-sd-d16  FloatingPointArchitektur

#-----------------------------------------------------------------------------

# Projektweit gueltige/notwendige Uebergabeparameter fuer Assembler
ASMFLAGS  = -mcpu=$(ARM_CORE) 
  #16-/32-Bit Funktionsaufrufe ermoeglichen
  ASMFLAGS += -mthumb-interwork -gstabs

# Projektweit gueltige/notwendige Uebergabeparameter fuer C-Compiler
CCFLAGS   = -mcpu=$(ARM_CORE) -mlong-calls 
  #16-/32-Bit Funktionsaufrufe ermoeglichen
  CCFLAGS  += -mthumb-interwork
  #Funktionen in eigene .text.xxx Section legen  (zwecks Wegoptimierung)
  CCFLAGS  += -ffunction-sections  
  #Stack Benutzung in spearater Datei (*.su) ausgeben
  CCFLAGS  += -fstack-usage
  #Compiler-Warnings einschalten (-Wextra gibt auch dann eine Warning, wenn 
  # in der Initialisierungsliste Strukturelemente fehlen, daher -Wno-missing-field-initializers
  # zum deaktivieren dieser Warning)
  CCFLAGS  += -Wall -Wextra -Wno-missing-field-initializers
  #Use colored compiler output
  CCFLAGS  += -fdiagnostics-color

# Projektweit gueltige/notwendige Uebergabeparameter fuer C-Compiler (THUMB-Mode)
CTFLAGS =
  #Automatische Sprachenzuordnung (*.c -> C) funktioniert bei .ct nicht mehr
  #daher den Compiler h ndisch auf die Sprache C setzen
  CTFLAGS += -x c

# Projektweit gueltige/notwendige Uebergabeparameter fuer Linker 
LINKFLAGS  = 
  #MAP-Datei erzeugen
  LINKFLAGS  = -Wl,-Map,$(BIN_DIR)/$(APP_NAME).map,--cref,--gc-sections
  #Pfadangabe in ELF-File aendern
  #LINKFLAGS +=  -fdebug-prefix-map=/mnt/c/_Projekte/nxt/demo/=. 
  #Speichernutzung ausgeben lassen
  LINKFLAGS += -Wl,--print-memory-usage
  #Do not expand any symbolic links, resolve references to `/../' or `/./', or make the path absolute when generating a relative prefix.
  #LINKFLAGS  += -no-canonical-prefixes
  #Umgang mit nicht im LinkerScript beschriebenen Sectionen
  #https://maskray.me/blog/2024-06-02-understanding-orphan-sections
  #LINKFLAGS += -Wl,--orphan-handling=warn
  #Warn if the linker creates a loadable, non-zero sized segment that has all three
  #of the read, write and execute permission flags set. Such a segment represents
  #a potential security vulnerability. In addition warnings will be generated if a
  #thread local storage segment is created with the execute permission flag set,
  #regardless of whether or not it has the read and/or write flags set.
  #https://www.redhat.com/en/blog/linkers-warnings-about-executable-stacks-and-segments
  #LINKFLAGS += -Wl,--no-warn-rwx-segment
  #LINKFLAGS += -nostdlib  Do not use the standard system startup files or libraries when linking
  #LINKFLAGS += -nolibc    Do not use the C library or system libraries tightly coupled with it when linking  
  #Linker auf libc_nano 'ummapen'
  LINKFLAGS += -specs=nano.specs

# Projektweit gueltige/notwendige Datei mit den Abh ngigkeiten der einzelnen Dateien
DEP_FLAG =
  #$(DEP_DIR)*.Po Datei erzeugen, in welche alle Abh ngigkeiten/Dependencies (include-Dateien) enthalten sind
  #siehe Anhang: GCC: Dependency Generation
  DEP_FLAG += -MT $@ -MD -MP -MF $(DEP_DIR)/$*.Tpo

#-----------------------------------------------------------------------------

# Projektspezifische Uebergabeparameter fuer Compiler
#Applikationsname als Makro uebergeben
CCFLAGS  += -DAPP_NAME=\"$(APP_NAME)\"

ifeq ($(PROJECT_TARGET),DEBUG)
  #Compiler verbieten, Funktionen sie memcpy() zu inlinen
  CCFLAGS  += -fno-builtin 
  #Debug-Informationen einschalten
  CCFLAGS  += -g
  #Optimierung auschalten
  CCFLAGS  += -O0
else ifeq ($(PROJECT_TARGET),RELEASE)
  #Makro fuer RELEASE Mode setzen
  CCFLAGS  += -DNDEBUG
  #Optimierung auf Codegroesse
  CCFLAGS += -Os
endif

ifeq ($(LST_FILES),YES)
  #Zwischencode (Assembler-Datei) als *.lst Datei ausgeben
  CCFLAGS += -Wa,-adhlns=$(basename $@).lst 
endif

#Ausgabe von SIZE 'formatieren'
SIZE_SED  = -e '/[.].*0x[0-9a-fA-F]*[ ]*0x0/d' #Zeilen mit mit einem Punkt anfangen und
                                               #einer StartAdresse von 0x0 entfernen (z.B. .comment 0x49 0x0)
SIZE_SED += -e '/^\s*$$/d'                #Leerzeilen entfernen
SIZE_SED += -e '/demo.elf/d'              #Headerzeile (bestehend aus dem Dateinamen) entfernen
SIZE_SED +=	-e 's/^/\t/'                  #Tabs vorne an Zeile anhaengen

#-----------------------------------------------------------------------------
# Automatisierungsregeln/-variablen zum Erstellen von Dateien auf BAsis von sed
# - LD_SED -> Linker Script File (mit Speicherbeschreibung)
#   Source:      $(TEMPLATE_INC)/link.ld
#   Destination: $(BIN_DIR)/$(APP_NAME).ld
# - CONFIGT32_SED -> Lauterbach Konfigurationsdatei
#   Source:      $(TEMPLATE_INC)/config.t32 
#   Destination: $(TRACE32_INSTALL_DIR)/config.t32
# - SYSTEMSETTINGS_SED -> Lauterbach Konfigurationsdatei (Startverzeichnis, Zusatzmenue, Hilfsausgabe)
#   Source:      $(TEMPLATE_INC)/system-settings.cmm
#   Destination: $(TRACE32_INSTALL_DIR)/system-settings.cmm
# - LOAD_SED -> Lauterbach Scrip zum Laden der Applikation
#   Source:      $(TEMPLATE_INC)/load.cmm
#   Destination: .
# - WINDOW_SED -> Lauterbach Scrip zum Positionieren der Fenster
#   Source:      $(TEMPLATE_INC)/window.cmm
#   Destination: .
# - OPENOCD_SED -> OpenOCD Konfigurationsdatei
#   Source:      $(OPENOCD_INC)/openOCD.cfg 
#   Destination: .
# - GDBINIT_SED -> GDB Konfigurationsdatein
#   Source:      $(OPENOCD_INC)/.gdbinit 
#   Destination: .

ifeq ($(MODE),$(filter $(MODE),TRACE32_ROM TRACE32_RAM TRACE32_SIM))
  CONFIGT32_SED = -e "s/__TRACE32_HEADER__/$(Configuration)/"
  CONFIGT32_SED+= -e 's|__TRACE32_INSTALL_DIR__|$(TRACE32_INSTALL_DIR)|'

  SYSTEMSETTINGS_SED  = -e 's|__WORKING_DIR__|$(shell pwd)|'

  LOAD_SED      = -e "s/__INPUT_FILE__/$(BIN_DIR)\/$(APP_NAME).elf/" 
  LOAD_SED     += -e 's|__TRACE32_PATH__|$(TRACE32_INC)|'
  LOAD_SED     += -e 's|__WORKING_DIR__|$(shell pwd)|'
  LOAD_SED     += -e 's|__TRACE32_INSTALL_DIR__|$(TRACE32_INSTALL_DIR)|'
  ifeq ($(TRACE32_OS),WINDOWS)
    LINUXPATH2WINPATH_SED  = -e 's|/mnt/c/|c:\\|'
    LINUXPATH2WINPATH_SED += -e 's|/mnt/d/|d:\\|'
    LINUXPATH2WINPATH_SED += -e 's|/mnt/e/|e:\\|'
  	CONFIGT32_ECHO      = "$(TRACE32_INSTALL_DIR)/bin/windows/t32marm.exe"
  else ifeq ($(TRACE32_OS),LINUX)
	CONFIGT32_ECHO      = "$(TRACE32_INSTALL_DIR)/bin/pc_linux64/t32marm.exe"
  else
	CONFIGT32_ECHO      = "$(TRACE32_INSTALL_DIR)/bin/macosx64/t32marm-qt.app"
  endif

  #Quelle: dvd/setup_linux.sh
  #if [ "$( uname -s )" = "Darwin" ]; then
  #	# In case this is a MacOSx installation
  #	HOSTSYSTEM="macosx64"
  #fi
  ## Try to determine host system in case of linux host
  #if [ -z "$HOSTSYSTEM" ]; then
  #	if [ "$(uname -m)" = "aarch64" ]; then
  #		HOSTSYSTEM="linux-aarch64"
  #	elif [ "$(getconf LONG_BIT)" = "64" ]; then
  #		HOSTSYSTEM="pc_linux64"
  #	else
  #		HOSTSYSTEM="pc_linux"
  #	fi
  #elif [ "$HOSTSYSTEM" = "macosx64" ]; then
  #	if [ -d "$INSTALLDIR"/bin/$HOSTSYSTEM ]; then
  #		#Existing MacOS installation -> Delete t32*.app directories first to ensure consistency
  #		rm -fr "$INSTALLDIR"/bin/$HOSTSYSTEM/t32*.app
  #	fi
  #fi  
endif

ifeq ($(MODE),TRACE32_ROM)
  APP_SRC_C     += 
  CCFLAGS       += -DMODE_ROM                #C-Makro 'MODE_ROM' setzen
  ASMFLAGS      += --defsym MODE_ROM=1       #ASM-Marko 'MODE_ROM=1' setzen
  LD_SED        += -e "s/^ROM_ONLY//"   -e "/^[a-zA-Z]*_ONLY/d"
  CONFIGT32_SED += -e "s/^ROM_ONLY //"  -e "/^[a-zA-Z]*_ONLY /d"
  ifeq ($(TRACE32_OS),WINDOWS)
    CONFIGT32_SED      += $(LINUXPATH2WINPATH_SED)  #Linux Pfadangabe nach WinPfadangabe konvertieren
    SYSTEMSETTINGS_SED += $(LINUXPATH2WINPATH_SED)
  endif
  LOAD_SED      += -e "s/^;ROM_ONLY //" -e "s/^;[A-Za-z]*_ONLY /;/"
  WINDOW_SED    += -e "s/^;ROM_ONLY //" -e "s/^;[A-Za-z]*_ONLY /;/"
  ALL_TARGETDEP  =  $(TRACE32_INSTALL_DIR)/config.t32 
  ALL_TARGETDEP +=  $(TRACE32_INSTALL_DIR)/system-settings.cmm 
  ALL_TARGETDEP +=  load.cmm window.cmm
  ALL_TARGETDEP +=  $(BIN_DIR)/$(APP_NAME).elf
else ifeq ($(MODE),TRACE32_RAM)
  APP_SRC_C     += $(TRACE32_INC)/udmon3
  CCFLAGS       += -DMODE_RAM
  ASMFLAGS      += --defsym MODE_RAM=1
  LD_SED        += -e "s/^RAM_ONLY//"   -e "/^[a-zA-Z]*_ONLY/d"
  CONFIGT32_SED += -e "s/^RAM_ONLY //"  -e "/^[a-zA-Z]*_ONLY /d"
  ifeq ($(TRACE32_OS),WINDOWS)
    CONFIGT32_SED      += $(LINUXPATH2WINPATH_SED)  #Linux Pfadangabe nach WinPfadangabe konvertieren
    SYSTEMSETTINGS_SED += $(LINUXPATH2WINPATH_SED)
  endif
  LOAD_SED      += -e "s/^;RAM_ONLY //" -e "s/^;[A-Za-z]*_ONLY /;/" 
  WINDOW_SED    += -e "s/^;RAM_ONLY //" -e "s/^;[A-Za-z]*_ONLY /;/" 
  ALL_TARGETDEP  =  $(TRACE32_INSTALL_DIR)/config.t32 
  ALL_TARGETDEP +=  $(TRACE32_INSTALL_DIR)/system-settings.cmm 
  ALL_TARGETDEP +=  load.cmm window.cmm
  ALL_TARGETDEP +=  $(BIN_DIR)/$(APP_NAME).elf
else ifeq ($(MODE),TRACE32_SIM)
  APP_SRC_C     += $(TRACE32_INC)/udmon3
  CCFLAGS       += -DMODE_SIM
  ASMFLAGS      += --defsym MODE_SIM=1
  LD_SED        += -e "s/^SIM_ONLY//"   -e "/^[a-zA-Z]*_ONLY/d"
  CONFIGT32_SED += -e "s/^SIM_ONLY //"  -e "/^[a-zA-Z]*_ONLY /d"
  ifeq ($(TRACE32_OS),WINDOWS)
    CONFIGT32_SED      += $(LINUXPATH2WINPATH_SED)  #Linux Pfadangabe nach WinPfadangabe konvertieren
    SYSTEMSETTINGS_SED += $(LINUXPATH2WINPATH_SED)
    LOAD_SED           += -e "s/^;SIMWINDOWS_ONLY //" 
  else ifeq ($(TRACE32_OS),LINUX)
    LOAD_SED           += -e "s/^;SIMLINUX_ONLY //" 
  else
    LOAD_SED           += -e "s/^;SIMMACOS_ONLY //" 
  endif
  LOAD_SED      += -e "s/^;SIM_ONLY //" -e "s/^;[A-Za-z]*_ONLY /;/"
  WINDOW_SED    += -e "s/^;SIM_ONLY //" -e "s/^;[A-Za-z]*_ONLY /;/"
  ALL_TARGETDEP  =  $(TRACE32_INSTALL_DIR)/config.t32 
  ALL_TARGETDEP +=  $(TRACE32_INSTALL_DIR)/system-settings.cmm 
  ALL_TARGETDEP +=  load.cmm window.cmm
  ALL_TARGETDEP +=  $(BIN_DIR)/$(APP_NAME).elf
else ifeq ($(MODE),SAMBA_RAM)
  APP_SRC_C     += 
  CCFLAGS       += -DMODE_SAMBA
  ASMFLAGS      += --defsym MODE_SAMBA=1
  LD_SED        += -e "s/^SAMBA_ONLY//"  -e  "/^[a-zA-Z]*_ONLY/d"
  ALL_TARGETDEP  =  $(APP_NAME).samba
else ifeq ($(MODE),GDBOPENOCD_RAM)
  APP_SRC_C     += $(OPENOCD_INC)/dcc_stdio.c
  CCFLAGS       += -DMODE_GDBOPENOCD_RAM
  ASMFLAGS      += --defsym MODE_GDBOPENOCD_RAM=1
  LD_SED        += -e "s/^RAMOPENOCD_ONLY//"  -e  "/^[a-zA-Z]*_ONLY/d"
  OPENOCD_SED    = -e "s/__INPUT_FILE__/$(BIN_DIR)\/$(APP_NAME).elf/" 
  OPENOCD_SED   += -e "s/__VECTOR_START__/0x$(VECTOR_START)/" 
  GDBINIT_SED    = -e "s/__INPUT_FILE__/$(BIN_DIR)\/$(APP_NAME).elf/" 
  GDBINIT_SED   += -e "s/__VECTOR_START__/0x$(VECTOR_START)/" 
  ALL_TARGETDEP  =  $(BIN_DIR)/$(APP_NAME).elf
  ALL_TARGETDEP +=  openOCD.cfg   #Nach hinten, andernfalls funktioniert die 
  ALL_TARGETDEP +=  .gdbinit      #Aufl sung von __VECTOR_START__ nicht!
endif

#-----------------------------------------------------------------------------
# Versions Abhaengigkeiten

#$(basename ...) Entfernt die Dateinamenserweitung
#  $(basename src/foo.c src-1.0/bar hacks) -> src/foo src-1.0/bar hacks
#$(suffix  ..) Gibt die Dateierweiterung zur ck
#  $(suffix src/foo.c src-1.0/bar.c hacks) -> .c .c
#$(notdir ...) Entfern die Pfadangabe
#  $(notdir src/foo.c hacks) -> foo.c hacks
#$(dir ...) Gibt die Pfadangabe zur ck
#  $(dir src/foo.c hacks) -> src/ ./
#$(addsuffix suffix,names...) H ngt an names den suffix an
#  $(addsuffix .c,foo bar) -> foo.c bar.c
#$(addprefix prefix,names...) fuer vor names den prefix an
#  $(addprefix src/,foo bar) -> src/foo src/bar
#$(join list1,list2) Elemente von List1 mit Elemente von List2 konkatenieren
#  $(join a b,.c .o) -> a.c b.o
#$(realpath names...) Erzeugt fuer jede Datei in Names den absoluten Dateinamen
#  $(realpath main.c) -> /mnt/c/_Projekte/nxt/demo/main.c 
#$(abspath names...) Wie realpath, nur dass keine symblinks aufgel st werden
#                    und Dateien nicht existieren m ssen


BIN_SRCS  =  $(addprefix $(BIN_DIR)/, $(notdir $(basename $(APP_SRC_ASM)))) \
	     	 $(addprefix $(BIN_DIR)/, $(notdir $(basename $(APP_SRC_C)))) 

#Suchpfade setzen, so dass '%.o : %.c' Quelldateien nicht nur im Arbeitsverzeichnis sucht
VPATH =  $(dir $(APP_SRC_ASM)) $(dir $(APP_SRC_C))

#-----------------------------------------------------------------------------
# Konfigurations/Debug-Ausgaben

Configuration = $(PROJECT_TARGET)-Purpose MODE=$(MODE) LST_FILES=$(LST_FILES)

#$(error/warning/info  Ausgabe werden vor der Bearbeitung des ersten Targets ausgegeben
$(info ---------------------------------------------)
$(info Target       :  $(BIN_DIR)/$(APP_NAME).elf)
$(info Configuration:  $(Configuration) )
$(info ---------------------------------------------)

#-----------------------------------------------------------------------------
# Abhaengigkeiten

#Target all 
# - Erste Regel im Makefile, welches genutzt wird, wenn make ohne Angabe einer Regel aufgerufen wird
# - Weitere Regel sind
#   all -> zum Erstellen des vollstaendigen Programms
#   clean -> zum Loeschen aller Zwischenergebnisse
#   install -> all + Kopieren der ausfuehrbaren Datei, der Librarys usw. an
#              die Positionen, wo sie 'erwartet' werden
#   uninstall -> Zum Loeschen der all mit install installierten Dateien
#   ... -> siehe https://www.gnu.org/software/make/manual/html_node/Standard-Targets.html
.PHONY: all
all: $(BIN_DIR)/.f $(DEP_DIR)/.f $(ALL_TARGETDEP)

$(BIN_DIR)/$(APP_NAME).elf : $(BIN_SRCS:=.o) $(TEMPLATE_INC)/link.ld
	@echo "$(RED)Linking$(NC) $(notdir $?) to $(notdir $@)"
	$(SED) $(LD_SED) $(TEMPLATE_INC)/link.ld >$(BIN_DIR)/$(APP_NAME).ld
	$(LINK) $(BIN_SRCS:=.o) $(SHLIBS) -T $(BIN_DIR)/$(APP_NAME).ld $(LINKFLAGS) -o $@
	$(SIZE) -A -x $@ | sed $(SIZE_SED)

$(BIN_DIR)/$(APP_NAME).bin : $(BIN_DIR)/$(APP_NAME).elf
	@echo "$(RED)Converting$(NC) $< to $@"
	$(BIN) -O binary -S $< $(BIN_DIR)/$(APP_NAME).bin

$(BIN_DIR)/$(APP_NAME).ihex : $(BIN_DIR)/$(APP_NAME).elf
	@echo "$(RED)Converting$(NC) $< to $@"
	$(BIN) -O ihex -S $< $(BIN_DIR)/$(APP_NAME).bin
	
$(APP_NAME).samba : $(BIN_DIR)/$(APP_NAME).bin
	@echo "$(RED)Converting$(NC) $< to $@ $(GREEN)(Startaddress=0x$(VECTOR_START))$(NC)"
	$(eval VECTOR_START:=$(shell $(NM) $(BIN_DIR)/$(APP_NAME).elf | $(SED) -e '/T[ ]*ivt_boot/!d' -e 's/[ ]*T[ ]*ivt_boot//'))
	$(BIN2SAMBA) $< $@ 0x$(VECTOR_START)
	@echo "type >>0x80 0x80 #    for Auto Baudrate Sequence"
	@echo "type >>V#             for Sam-Ba Version"
	@echo "type >>O/H/Wadr,valu# for Writing Byte/Half/Word"
	@echo "type >>o/h/wadr#      for Reading Byte/Half/Word"
	@echo "type >>g0x$(VECTOR_START)#     to start Application"

$(TRACE32_INSTALL_DIR)/config.t32: $(TEMPLATE_INC)/config.t32 makefile
	@echo "$(RED)Creating$(NC) $@ from $< for TRACE32-Start $(GREEN)($(CONFIGT32_ECHO))$(NC)"
#	@echo "$(RED)Creating$(NC) $@ from $< for TRACE32-Start $(GREEN)($(CONFIGT32_ECHO) -c config.t32)$(NC)"
	$(SED) $(CONFIGT32_SED) $< >$@

$(TRACE32_INSTALL_DIR)/system-settings.cmm: $(TEMPLATE_INC)/system-settings.cmm makefile
	@echo "$(RED)Creating$(NC) $@ from $< for TRACE32-Start"
	$(SED) $(SYSTEMSETTINGS_SED) $< >$@

load.cmm: $(TEMPLATE_INC)/load.cmm makefile
	@echo "$(RED)Creating$(NC) $@ from $< for TRACE32-CLI $(GREEN)(do load)$(NC)"
	$(SED) $(LOAD_SED) $< >$@  

window.cmm: $(TEMPLATE_INC)/window.cmm makefile
	@echo "$(RED)Creating$(NC) $@ from $< for TRACE32-Windows $(GREEN)(do window)$(NC)"
	$(SED) $(WINDOW_SED) $< >$@  

openOCD.cfg: $(OPENOCD_INC)/openOCD.cfg makefile
	@echo "$(RED)Creating$(NC) $@ from $< for openOCD"
	$(eval VECTOR_START:=$(shell $(NM) $(BIN_DIR)/$(APP_NAME).elf | $(SED) -e '/T[ ]*ivt_boot/!d' -e 's/[ ]*T[ ]*ivt_boot//'))
	$(SED) $(OPENOCD_SED) $< >$@

.gdbinit: $(OPENOCD_INC)/.gdbinit makefile
	@echo "$(RED)Creating$(NC) $@ from $< for gdb"
	$(eval VECTOR_START:=$(shell $(NM) $(BIN_DIR)/$(APP_NAME).elf | $(SED) -e '/T[ ]*ivt_boot/!d' -e 's/[ ]*T[ ]*ivt_boot//'))
	$(SED) $(GDBINIT_SED) $< >$@

#Im $(DEP_DIR) Verzeichnis sind *.Po enthalten (erzeugt ueber $(DEP_FLAG))
# in welcher fuer jede C-Datei die Abhaengigkeiten (im makefile Syntax) stehen
# so dass bei Aenderung einer der Abhaengigkeiten die passenden Regeln 'angeworfen' werden
-include $(addprefix ./$(DEP_DIR)/,$(notdir $(BIN_SRCS:=.Po)))

$(BIN_DIR)/%.o: %.s makefile 
	@echo "$(RED)ARM-Assembling$(NC)  $< to $@"
	$(ASM) $(ASMFLAGS) -o$@ $<

$(BIN_DIR)/%.o : %.c makefile
	echo "$(RED)ARM-Compiling$(NC) $< to $@"
	$(CC) -marm  $(CCFLAGS) $(CCINC) $(DEP_FLAG) -c -o$@ $<
	$(MV) -f $(DEP_DIR)/$*.Tpo $(DEP_DIR)/$*.Po
	
$(BIN_DIR)/%.o : %.ct makefile
	@echo "$(RED)Thumb-Compiling$(NC) $< to $@"
	$(CC) -mthumb $(CCFLAGS) $(CTFLAGS) $(CCINC) $(DEP_FLAG) -c -o$@ $<
	$(MV) -f $(DEP_DIR)/$*.Tpo $(DEP_DIR)/$*.Po
	
#Anweisung, welche besagt, dass hinter den besagten Targets keine Dateien existieren
.PHONY : clean
clean: 
	@echo "$(RED)Deleting $(NC) $(BIN_DIR)- und $(DEP_DIR)- Directorys"
	$(RM) $(BIN_DIR)
	$(RM) $(DEP_DIR)

#Anweisung, damit make tempor re Dateien nicht l scht (hier .f)
.PRECIOUS: %/.f
%/.f:
	@echo "$(RED)MkDir$(NC) $(dir $@)"
	$(MKDIR) $(@D)
	$(TOUCH) $@

help:
	# filter all targets starting with lowercase and containing lowercase letters, hyphens, or underscores;
	# Quelle: Riot/makefile.include
	make -qp | sed -ne 's/\(^[a-z][a-z_-]*\):.*/\1/p' | sort -u
	# filter all targets starting with lowercase and containing lowercase letters, hyphens, or underscores; explicitly include generate-Makefile.ci
	#@$(MAKE) -qp | sed -ne 's/\(^[a-z][a-z_-]*\|generate-Makefile.ci\):.*/\1/p' | sort -u

#-------------------------------------------------------------------------------
# Anhang
#-------------------------------------------------------------------------------
# Check given command is available in the path
#   check_cmd 'command' 'description'
#	$(call check_cmd,$(CC),Compiler)
define check_cmd
	@command -v $1 >/dev/null 2>&1 || \
	  { $(COLOR_ECHO) \
	    '$(COLOR_RED)$2 $1 is required but not found in PATH.  Aborting.$(COLOR_RESET)'; \
	    exit 1;}
endef

#-------------------------------------------------------------------------------
# Variablenzuweisung 
#   - Simple assignment :=
#     Zuweisung wird nur einmalig (bei der ersten Auftreten) ausgef hrt.
#     Wird nach der Zuweisung eine SubVariable ge ndert CC:=${GCC}, so beh lt
#     die Variable ihren erstmaligen Wert bei
#   - Recursive assignment =
#     Zuweisung wird bei jeder Nutzung der zugewiesen Variablen ausgef hrt
#     Wird zwischen zwei Nutzungen eine Variable ge ndert CC=${GCC}, so  ndert 
#     sich auch der Zuweisungswert
#   - Conditional assignment ?=
#     Zuweisung erfolgt nur dann, wenn Variable noch keinen Wert hat
#   - Appending +=
#     String Concatenation, entspricht CC = ${CC} xyz 

#-------------------------------------------------------------------------------
# Make: Automatic Variables
# MAKEFLAGS make automatically passes down variable values that were defined on the command line, 
#           by putting them in the MAKEFLAGS variable. See Options/Recursion.
# -xxx      Ignore Error
# @xxx      Supress Echo
# $@        The file name of the target of the rule. If the target is an archive member, 
#           then `$@' is the name of the archive file. In a pattern rule that has multiple 
#           targets, `$@' is the name of whichever target caused the rule's commands to be run. 
# $%        The target member name, when the target is an archive member. 
#           For example, if the target is foo.a(bar.o) then '$%' is bar.o and '$@' is foo.a. 
#           '$%' is empty when the target is not an archive member. 
# $<        The name of the first prerequisite. If the target got its commands from an implicit rule, 
#           this will be the first prerequisite added by the implicit rule (see Implicit Rules). 
# $?        The names of all the prerequisites that are newer than the target, with spaces between them
#           For prerequisites which are archive members, only the member named is used (see Archives). 
# $^        The names of all the prerequisites, with spaces between them. For prerequisites which are 
#           archive members, only the member named is used. A target has only one prerequisite 
#           on each other file it depends on, no matter how many times each file is listed as a 
#           prerequisite. So if you list a prerequisite more than once for a target, the value 
#           of '$^' contains just one copy of the name. This list does not contain any of the 
#           order-only prerequisites; for those see the `$|' variable, below. 
# $+        This is like '$^', but prerequisites listed more than once are duplicated in the 
#           order they were listed in the makefile. This is primarily useful for use in linking 
#           commands where it is meaningful to repeat library file names in a particular order. 
# $|        The names of all the order-only prerequisites, with spaces between them. 
# $*        The stem with which an implicit rule matches. If the target is dir/a.foo.b and the 
#           target pattern is a.%.b then the stem is dir/foo. The stem is useful for constructing 
#           names of related files. In a static pattern rule, the stem is part of the file name
#           that matched the '%' in the target pattern. 
#           In an explicit rule, there is no stem; so `$*' cannot be determined in that way. 
#           Instead, if the target name ends with a recognized suffix, '$*' is set to the target
#           name minus the suffix. For example, if the target name is `foo.c', then `$*' is set 
#           to `foo', since `.c' is a suffix. GNU make does this bizarre thing only for compatibility 
#           with other implementations of make. You should generally avoid using `$*' except in 
#           implicit rules or static pattern rules. 
#           If the target name in an explicit rule does not end with a recognized suffix, '$*' is 
#           set to the empty string for that rule. 
# $?        is useful even in explicit rules when you wish to operate on only the prerequisites that
#           have changed. For example, suppose that an archive named lib is supposed to contain copies 
#           of several object files. This rule copies just the changed object files into the archive: 
#             lib: foo.o bar.o lose.o win.o
#                 ar r lib $?
#           Of the variables listed above, four have values that are single file names, and three have 
#           values that are lists of file names. These seven have variants that get just the file's 
#           directory name or just the file name within the directory. The variant variables' names 
#           are formed by appending `D' or `F', respectively. These variants are semi-obsolete 
#           in GNU make since the functions dir and notdir can be used to get a similar effect. 
#           Note, however, that the `D' variants all omit the trailing slash which always appears 
#           in the output of the dir function. Here is a table of the variants: 
# $(@D)     The directory part of the file name of the target, with the trailing slash removed. If the 
#           value of `$@' is dir/foo.o then `$(@D)' is dir. This value is . if `$@' does not contain a slash. 
# $(@F)     The file-within-directory part of the file name of the target. If the
#           value of '$@' is dir/foo.o then '$(@F)' is foo.o. '$(@F)' is equivalent to `$(notdir $@)'. 
# $(*D) $(*F) The directory part and the file-within-directory part of the stem; dir and foo in this example. 
# $(%D) $(%F) The directory part and the file-within-directory part of the target archive member name. 
#             This makes sense only for archive member targets of the form archive(member) and is useful 
#             only when member may contain a directory name.
# $(<D) $(<F) The directory part and the file-within-directory part of the first prerequisite. 
# $(^D) $(^F) Lists of the directory parts and the file-within-directory parts of all prerequisites. 
# $(+D) $(+F) Lists of the directory parts and the file-within-directory parts of all prerequisites, 
#             including multiple instances of duplicated prerequisites. 
# $(?D) $(?F) Lists of the directory parts and the file-within-directory parts of all prerequisites that
#             are newer than the target.

#-------------------------------------------------------------------------------
# GCC: Dependency Generation
#-M          Instead of outputting the result of preprocessing, output a rule 
#            suitable for make describing the dependencies of the main source file. 
#            The preprocessor outputs one make rule containing the object file name 
#            for that source file, a colon, and the names of all the included 
#            files, including those coming from  -include  or  -imacros  command line options.
# -MT target Change the target of the rule emitted by dependency generation.  
#            By default CPP takes the name of the main input file, deletes 
#            any directory components and any file suffix such as  .c , 
#            and appends the platform s usual object suffix.
# -MF file   When used with  -M  or  -MM , specifies a file to write the dependencies to. 
#            If no  -MF  switch is given the preprocessor sends the rules to the same 
#            place it would have sent preprocessed output.
#            When used with the driver options  -MD  or  -MMD ,  -MF  overrides the default
#            dependency output file.
# -MP        This option instructs CPP to add a phony target for each dependency 
#            other than the main file, causing each to depend on nothing. These 
#            dummy rules work around errors make gives if you remove header files 
#            without updating the  Makefile  to match.
# -MD        is equivalent to  -M -MF file , except that  -E  is not implied.
#            The driver determines file based on whether an  -o  option is given. 
#            If it is, the driver uses its argument but with a suffix of  .d , 
#            otherwise it takes the name of the input file, removes any directory 
#            components and suffix, and applies a  .d  suffix. 
#            If  -MD  is used in conjunction with  -E , any  -o  switch is understood 
#            to specify the dependency output file (see [-MF], page 134), but if used
#            without  -E , each  -o  is understood to specify a target object file.
#            Since  -E  is not implied,  -MD  can be used to generate a dependency output
#            file as a side-effect of the compilation process.
# -MMD       Like  -MD  except mention only user header files, not system header files.
# -H         anzeigen der genutzten Include-Pfade
# -mno-cygwin Nutzung der MINGW/INclude Files und Nutzung der OS-Spezifischen DLL's anstatt CYGWIN1.DLL
#             http://www.delorie.com/howto/cygwin/mno-cygwin-howto.html
