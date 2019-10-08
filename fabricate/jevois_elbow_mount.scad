inch = 25.4;
W = 32;
H = 36;
w = 1 * inch;
h = 1.175 * inch;

T = 4;

extend = 0;
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
  translate([0, 0, 0])rotate([0, 0, 0])difference(){
    translate([0, 0, 0])base();
    translate([0, 0+extend, 0])
      union(){
      translate([-51,   67 + offset, -20])cylinder(r=23.8 + offset, h=50, $fn=30);
      translate([-51.1, 67 + offset, 40])sphere(r=51.5, $fn=50);
    }
  }

}
difference(){
  union(){
    translate([51.1, -20, 0])rotate([0, 0, 0])unit();
    //translate([-35/2, 0, 0])cube([35, 5, 45]);
    
    translate([0, T, 10])rotate(a=90, v=[1, 0, 0])translate([0, H/2, 0])difference(){
      translate([-W/2, -H/2 - T, 0])cube([W, H + T, T]);
      translate([-w/2, -h/2, ,-1])cylinder(h=10, r=1.5, $fn=50);
      translate([-w/2,  h/2, ,-1])cylinder(h=10, r=1.5, $fn=50);
      translate([ w/2, -h/2, ,-1])cylinder(h=10, r=1.5, $fn=50);
      translate([ w/2,  h/2, ,-1])cylinder(h=10, r=1.5, $fn=50);
      translate([-w/2, -h/2, ,-7])cylinder(h=10, r=2.5, $fn=50);
      translate([-w/2,  h/2, ,-7])cylinder(h=10, r=2.5, $fn=50);
      translate([ w/2, -h/2, ,-7])cylinder(h=10, r=2.5, $fn=50);
      translate([ w/2,  h/2, ,-7])cylinder(h=10, r=2.5, $fn=50);
      translate([-50, -H/2 + 30 - 1, -18])cube([100, 50, 20]);
    }
  }
translate([0, -5, -5])rotate([0, 0, 0])cylinder(d1=40, d2=5, h=15);
}

