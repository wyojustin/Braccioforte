include<stereopi_braccio_mount.scad>

/*
translate([-51, 97, 16])
rotate([0, -90, 180])
color("red")import("Elbow.STL");
*/

extend = 15;
module base(){
  translate([-51, 97, 16])
    rotate([0, -90, 180])
    union(){
    difference()
      {
	translate([-19, 50-extend, -35/2])cube([10, 27.+extend, 35]);
	translate([0, -extend, 0])
	union(){
	  translate([-49, 62, 11])rotate([0, 90, 0])cylinder(d=3.5, h=50, $fn=30);
	  translate([-13.5, 62, 11])rotate([0, 90, 0])cylinder(d=8, h=50, $fn=30);
	  translate([-49, 62, -11])rotate([0, 90, 0])cylinder(d=3.5, h=50, $fn=30);
	  translate([-13.5, 62, -11])rotate([0, 90, 0])cylinder(d=8, h=50, $fn=30);
	  translate([-13.5, 59.8, 10.3])rotate([0, 90, 0])cylinder(d=15, h=50, $fn=30);
	  translate([-13.5, 59.8, -10.3])rotate([0, 90, 0])cylinder(d=15, h=50, $fn=30);
	  translate([-13.5, 17, -12.])cube([40, 50, 24]);
	  translate([-13.5, 40, -20.])cube([20, 20, 40]);
	}
      }
  }
}
module unit(){
  offset = 7;
  translate([0, 0, -5])rotate([15, 0, 0])difference(){
    translate([0, 10, 0])
      base();
    translate([0, 10+extend, 0])
      union(){
      color("blue")translate([-51, 67 + offset, -20])
	cylinder(r=23.8 + offset, h=50, $fn=30);
      translate([-51.1, 67 + offset, 40])sphere(r=51.5, $fn=50);
    }
  }

}
translate([0, 0, -3])rotate([0, 0, 0])
unit();
