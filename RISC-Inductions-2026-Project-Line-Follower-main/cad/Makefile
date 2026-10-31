OPENSCAD ?= openscad
OUT := out
SRC := robot.scad

.PHONY: all stls preview pose check clean

all: stls preview pose check

stls: $(OUT)/chassis.stl $(OUT)/wheel.stl $(OUT)/chassis_collision.stl $(OUT)/wheel_collision.stl

$(OUT):
	mkdir -p $(OUT)

$(OUT)/chassis.stl: $(SRC) | $(OUT)
	$(OPENSCAD) -o $@ -D part_id=1 -D show_pose=0 $(SRC)

$(OUT)/wheel.stl: $(SRC) | $(OUT)
	$(OPENSCAD) -o $@ -D part_id=2 -D show_pose=0 $(SRC)

$(OUT)/chassis_collision.stl: $(SRC) | $(OUT)
	$(OPENSCAD) -o $@ -D part_id=3 -D show_pose=0 $(SRC)

$(OUT)/wheel_collision.stl: $(SRC) | $(OUT)
	$(OPENSCAD) -o $@ -D part_id=4 -D show_pose=0 $(SRC)

preview: $(OUT)/preview.png

$(OUT)/preview.png: $(SRC) | $(OUT)
	$(OPENSCAD) -o $@ --imgsize=1400,1050 --projection=p --autocenter --viewall \
		--camera=0,0,0,58,0,26,560 -D part_id=0 -D show_pose=0 $(SRC)

pose: $(OUT)/pose.echo

$(OUT)/pose.echo: $(SRC) | $(OUT)
	$(OPENSCAD) -o $@ -D part_id=5 -D show_pose=1 $(SRC)

check: stls
	python3 check_stl.py $(OUT)/chassis.stl $(OUT)/wheel.stl \
		$(OUT)/chassis_collision.stl $(OUT)/wheel_collision.stl

clean:
	rm -rf $(OUT)
