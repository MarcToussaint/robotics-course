
ipynb_paths =  $(shell find . -type f -name '*.ipynb' -not -name '*checkpoint*' -printf "%f ")

run:
	@for p in $(ipynb_paths); do jupyter-nbconvert --to notebook --execute $$p; done

clean: $(ipynb_paths:%=clean/%)
runAll: $(ipynb_paths:%=run/%)

clean/%.ipynb: %.ipynb
	+@-jupyter-nbconvert --clear-output --inplace $<

run/%.ipynb: %.ipynb
	+@-echo "=========== run $< ======="
	+@-jupyter-nbconvert --to notebook --execute $<
