# VentCon2 LaTeX Documentation Makefile
# Requires: pdflatex, tikz, pgfplots packages

# Document names
MAIN_DOC = VentCon2_Documentation
DEPS_DOC = Dependencies_Matrix

# Commands
LATEX = pdflatex
VIEWER = start  # Windows command to open PDF

# Default target
all: $(MAIN_DOC).pdf $(DEPS_DOC).pdf

# Build main documentation
$(MAIN_DOC).pdf: $(MAIN_DOC).tex
	$(LATEX) $(MAIN_DOC).tex
	$(LATEX) $(MAIN_DOC).tex  # Second run for TOC and references

# Build dependencies matrix
$(DEPS_DOC).pdf: $(DEPS_DOC).tex
	$(LATEX) $(DEPS_DOC).tex

# Clean auxiliary files
clean:
	del /f /q *.aux *.log *.toc *.out *.fdb_latexmk *.fls *.synctex.gz 2>nul || echo No aux files to clean

# Clean everything including PDFs
cleanall: clean
	del /f /q *.pdf 2>nul || echo No PDF files to clean

# View PDFs (Windows)
view: $(MAIN_DOC).pdf
	$(VIEWER) $(MAIN_DOC).pdf

view-deps: $(DEPS_DOC).pdf
	$(VIEWER) $(DEPS_DOC).pdf

# Help
help:
	@echo Available targets:
	@echo   all       - Build both documents
	@echo   main      - Build main documentation only
	@echo   deps      - Build dependencies matrix only
	@echo   clean     - Remove auxiliary files
	@echo   cleanall  - Remove all generated files
	@echo   view      - Open main documentation
	@echo   view-deps - Open dependencies matrix
	@echo   help      - Show this help

# Aliases
main: $(MAIN_DOC).pdf
deps: $(DEPS_DOC).pdf

.PHONY: all clean cleanall view view-deps help main deps
