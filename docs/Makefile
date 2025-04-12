# LATEX_CMD=xelatex
LATEX_CMD=pdflatex
DOCUMENT_NAME=filename
TEX_FILE=$(DOCUMENT_NAME).tex

all: $(TEX_FILE)
	$(LATEX_CMD) $(TEX_FILE)
	$(LATEX_CMD) $(TEX_FILE)

rename:
	@if [ -z "$(NAME)" ]; then \
		echo "Укажите новое имя файла: make rename NAME=new_filename"; \
		exit 1; \
	fi
	mv $(DOCUMENT_NAME).tex $(NAME).tex
	sed -i "s/$(DOCUMENT_NAME)/$(NAME)/g" $(NAME).tex

clean:
	rm -f *.ps *.dvi *.aux *.toc *.idx *.ind *.ilg *.log *.out *.brf *.blg *.bbl *.fdb_latexmk *.fls *.synctex.gz *.upa *.upb
