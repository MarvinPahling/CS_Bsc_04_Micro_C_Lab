# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a Typst-based academic documentation project for a Linux device driver lab assignment (Ostfalia University, µC-Peripherie course). The document covers Linux peripheral access via I2C, GPIO, and ADC interfaces.

## Build Commands

```bash
# Compile the document once
make compile

# Watch for changes and auto-compile
make watch

# Open PDF viewer (zathura)
make show

# Compile, show, and watch
make all
```

Output is generated to `./dist/main.pdf` from `./src/main.typ`.

### Code Examples (in src/figures/code/)

```bash
# Compile and run libgpiod example
make -C src/figures/code 01-compile
make -C src/figures/code 01-run

# Compile and run fan control example (requires sudo)
make -C src/figures/code 02-compile
make -C src/figures/code 02-run
```

## Architecture

### Document Structure
- `src/main.typ` - Main Typst document with all content
- `src/config/text-content.yaml` - Document metadata (title, authors, labels)
- `src/config/bibliography.bib` - Bibliography entries
- `src/figures/code/` - C code examples for embedding in document
- `src/figures/code/index.json` - Code snippet registry (id, path, description, lang)

### Code Snippet System
The document uses a custom `render-snippet(id)` function to embed code examples. Snippets are registered in `index.json` and referenced by ID. To add a new snippet:
1. Add the C file to `src/figures/code/`
2. Register it in `src/figures/code/index.json`
3. Use `#render-snippet("your-id")` in main.typ

### Typst Packages Used
- `finite` - Automaton diagrams
- `glossarium` - Glossary management
- `circuiteria` - Circuit diagrams
- `zebraw` - Code block styling with line numbers
