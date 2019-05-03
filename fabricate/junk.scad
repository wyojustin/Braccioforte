UP = 20;
inch = 25.4;
module flange_nut(d, D, h, H, UP){
  translate([0, 0, UP]){
    translate([0, 0, -.01])cylinder(h=h + .01, d=D, $fn=50);//flange
    translate([0, 0, h])cylinder(h=1, d1=D, d2=D/2, $fn=50);//flange
    cylinder(h=H+.01, d=d, $fn=6);//nut
  }
  cylinder(d=D, h=UP, $fn=50);
}

difference(){
  union(){
    cube([30, 6, 30]);
    translate([0, -24, 0])cube([30, 30, 6]);
  }
  translate([15,-20.1,15])rotate([-90, 0, 0])
    union(){
    scale([1.08, 1.08, 1])
      flange_nut(d=7 * inch/16.*1.05, D=19*inch/32*1.05, h=1.5, H=6, UP=UP);
    cylinder(d=.3*inch, h=100, $fn=50);
  }
translate([15-11, -20, -1])
rotate([0, 0, 0])cylinder(d=3.5, h=50, $fn=30);
translate([15-11+22, -20, -1])
rotate([0, 0, 0])cylinder(d=3.5, h=50, $fn=30);
}


