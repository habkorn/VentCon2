# VENTCON Web Interface Manual - Makefile
# Build with: make
# Clean with: make clean

TEX = pdflatex
MAIN = VENTCON_WebInterface_Manual

.PHONY: all clean

all: $(MAIN).pdf

$(MAIN).pdf: $(MAIN).tex
	$(TEX) $(MAIN).tex
	$(TEX) $(MAIN).tex

clean:
	rm -f $(MAIN).aux $(MAIN).log $(MAIN).out $(MAIN).toc $(MAIN).fdb_latexmk $(MAIN).fls $(MAIN).synctex.gz
