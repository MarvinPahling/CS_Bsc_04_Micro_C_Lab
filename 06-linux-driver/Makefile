OUT-PATH = ./dist/main.pdf
IN-PATH = ./src/main.typ
watch:
	typst watch $(IN-PATH) $(OUT-PATH)

compile:
	typst compile $(IN-PATH) $(OUT-PATH)

show:
	zathura $(OUT-PATH) &

all: compile show watch

.PHONY: all watch compile show
