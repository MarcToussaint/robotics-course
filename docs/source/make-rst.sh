#cat ../rst-header.tex > z.tex
#cat ${1%.*}.tex >> z.tex
#pandoc -f latex-smart z.tex --ascii -o ${1%.*}.rst

pandoc getting_started.md -o getting_started.rst
