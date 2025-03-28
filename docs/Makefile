# LATEX_CMD=xelatex
LATEX_CMD=pdflatex
DOCUMENT_NAME=AlexandrovMA
TEX_FILE=$(DOCUMENT_NAME).tex

copy:
	cp /home/maxim/study/AMIIMNASU/second/state_graph.png /home/maxim/study/AMIIMNASU/second/docs/images/state_graph.png
	cp /home/maxim/study/AMIIMNASU/second/states_probabilities.png /home/maxim/study/AMIIMNASU/second/docs/images/states_probabilities.png
	cp /home/maxim/study/AMIIMNASU/second/reliability_function.png /home/maxim/study/AMIIMNASU/second/docs/images/reliability_function.png
	cp /home/maxim/study/AMIIMNASU/second/state_trajectories.png /home/maxim/study/AMIIMNASU/second/docs/images/state_trajectories.png

all: copy $(TEX_FILE)
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
