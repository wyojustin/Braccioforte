W = 75;

cam_l = 25;
cam_w = 24;
cam_h = 1;
hole_l = 21.5;
hole_w = 13;
hole_inset = (cam_l - hole_l) / 2;

m15_diam = 1.7;
m2_diam = 1.9;
screw_wall_t = 1.2 * 2;

module screw_hole(H, D, d, counter_sink){
  rotate([180, 0, 0])
  difference(){
    cylinder(d=D, h=H, $fn=20);
    translate([0, 0, -1])cylinder(d=d, h=H + 2, $fn=20);
    if(counter_sink){
      translate([0, 0, H - 1.2])cylinder(d1=1, d2=D, h=2, $fn=20);
    }
  }
}

module raspicam(){
  l = cam_l;
  w = cam_w;
  h = cam_h;
  difference(){
    color([.1, .1, .1])cube([l, w, h]); //pcb
    translate([hole_inset, w - hole_inset, -50])cylinder(d=2, h=100, $fn=20);
    translate([w - hole_inset, w - hole_inset, -50])
      cylinder(d=2, h=100, $fn=20);
    translate([w - hole_inset, w - hole_inset - hole_w, -50])
      cylinder(d=2, h=100, $fn=20);
    translate([hole_inset, w - hole_inset - hole_w, -50])
      cylinder(d=2, h=100, $fn=20);
  }
  color([1, 0, 0])translate([l/2 - 8.5/2, w/2 - 8.5/2, h])cube([8.5, 8.5, 1]);
}
module cam_holes(){
  l = cam_l;
  w = cam_w;
  h = cam_h;
  translate([0, 0, 2.0]){
    translate([hole_inset, w - hole_inset, 0])
      screw_hole(4, m15_diam + screw_wall_t, m15_diam, true);
    translate([w - hole_inset, w - hole_inset, 0])
      screw_hole(4, m15_diam + screw_wall_t, m15_diam, true);
    translate([w - hole_inset, w - hole_inset - hole_w, 0])
      screw_hole(4, m15_diam + screw_wall_t, m15_diam, true);
    translate([hole_inset, w - hole_inset - hole_w, 0])
      screw_hole(4, m15_diam + screw_wall_t, m15_diam, true);
  }
}

module cam_drills(){// drill through holes
  l = cam_l;
  w = cam_w;
  h = cam_h;
  translate([0, 0, -10.0]){
    translate([hole_inset, w - hole_inset, 0])
      cylinder(d=m15_diam, h=30, $fn=30);
    translate([w - hole_inset, w - hole_inset, 0])
      cylinder(d=m15_diam, h=30, $fn=30);
    translate([w - hole_inset, w - hole_inset - hole_w, 0])
      cylinder(d=m15_diam, h=30, $fn=30);
    translate([hole_inset, w - hole_inset - hole_w, 0])
      cylinder(d=m15_diam, h=30, $fn=30);
  }
}

module support(){
  d = 2.15;
  difference(){
    translate([-1, 0, 0])cube([2, 10, 10]);
    translate([0, 11, 0])rotate([-40, 0, 0])translate([-4, -15, 0])cube([8, 14, 14]);
    cylinder(d=d, h=14, $fn=20);
  }
}
module stereo_pi_holes(){
  translate([0, 0, 0])rotate([180, 0, 0])
    screw_hole(10, m2_diam + screw_wall_t, m2_diam, true);
  translate([0, 0, 0])support();

  translate([85, 0, 0])rotate([180, 0, 0])
    screw_hole(10, m2_diam + screw_wall_t, m2_diam, true);
  translate([85, 0, 0])support();

  translate([85, 35, 0])rotate([180, 0, 0])
    screw_hole(10, m2_diam + screw_wall_t, m2_diam, true);
  translate([85, 35, 0])support();

  translate([0, 35, 0])rotate([180, 0, 0])
    screw_hole(10, m2_diam + screw_wall_t, m2_diam, true);
  translate([0, 35, 0])support();
}

module case(){
  // stiffeners
  /*
    translate([cam_l + 6, 1, -2])cube([2, cam_l + 4, 3]);
    translate([W - 6, 1, -2])cube([2, cam_l + 4, 3]);
    translate([cam_l + 6, 3, -2])rotate([0, 0, -57])cube([2, 48, 3]);
    translate([W - 4, 0, -2])rotate([0, 0, 57])cube([2, 48, 3]);
  */
  difference(){
    union(){
      translate([-1, 0, -2])cube([W + cam_l + 5, cam_w+4, 5]);
    }
    translate([2, 2, 0])cam_drills();
    translate([2 + W, 2, 0])cam_drills();
    translate([2 + 20.5, 2, 0])cam_drills();
    translate([2 + W - 20.5, 2, 0])cam_drills();
    
    translate([0, 2, -3])cube([W + 28, 25, 5]);
    //translate([2+cam_l/2-5, 2+cam_w/2-5, -50])cube([10, 10, 100]);
    //translate([2+cam_l/2-5, cam_w/2-5, -50])cube([10, 10, 100]);
    translate([1.5+cam_l/2-5, cam_w/2-5.5, -50])cube([10, 10, 100]);
    translate([1.5+cam_l/2-5 + 20.5, cam_w/2-5.5, -50])cube([10, 10, 100]);

    //translate([W + (2+cam_l/2-5), 2+cam_w/2-5, -50])cube([10, 10, 100]);
    translate([W + (1.5+cam_l/2-5), cam_w/2-5.5, -50])cube([10, 10, 100]);
    translate([W + (1.5+cam_l/2-5) - 20.5, cam_w/2-5.5, -50])cube([10, 10, 100]);
    //translate([30, -30, -10])cube([100, 100, 100]);
  }
  
  //translate([2, 2, -10])raspicam();
  translate([2, 2, 0])cam_holes();
  //translate([2 + W, 2, -10])raspicam();
translate([2 + W, 2, 0])cam_holes();
translate([2 + 20.5, 2, 0])cam_holes();
translate([2 + W - 20.5, 2, 0])cam_holes();

  cx = (W + cam_l + 4)/2;
  translate([-1, cam_w + 4, -50+3])cube([W + cam_l + 5, 2, 50]);
  translate([-1 + cx - 85/2, cam_w + 4.5, -45])rotate([90, 0, 0])
    stereo_pi_holes();
}
rotate([0, 180, 0])
intersection(){
 case();
 //translate([-5, 0, -3])cube([11, 100, 100]);
}
/*
*/

//// tripod mount
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


module side_support(){
  translate([1, 0, 0])rotate([0, -90, 0])linear_extrude(height=1)
    polygon([[0, 0], [45, cam_w + 4], [0, cam_w + 4]]);
}
side_support();
translate([-W - cam_l - 4, 0, 0]) side_support();
