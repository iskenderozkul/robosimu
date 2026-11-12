.PHONY: test validate figures scenarios report tune rebuild clean

MODEL := models/six_axis_arm.json

validate:
	python -m robosimu validate $(MODEL)

test:
	python -m unittest discover -s tests -v

figures:
	python -m robosimu simulate $(MODEL) --controller governor --figure figures/arm_reach.png --gif figures/arm_reach.gif
	python -m robosimu html $(MODEL) --out figures/arm_viewer.html

tune:
	python scripts/tune_baselines.py --jobs 8

scenarios:
	python -m robosimu scenarios $(MODEL) --out reports --seeds 5 --reaches 6 --jobs 8

rebuild:
	python scripts/rebuild_reports.py --out reports

report: scenarios
	python -m robosimu motor-compare $(MODEL) --out reports
	python -m robosimu gravity-map $(MODEL) --out reports
	python -m robosimu point-motion $(MODEL) --out reports

clean:
	remove-item -force -recurse __pycache__, robosimu/__pycache__, tests/__pycache__ -ErrorAction SilentlyContinue
