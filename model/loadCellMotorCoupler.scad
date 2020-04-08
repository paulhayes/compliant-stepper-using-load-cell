motorShaftDim = [5,3,6];
motorShaftBase = [5,2];

loadCell = [12.5+0.1,80,12.5];

case = [loadCell.x+5,46,loadCell.z];
caseBackLengthFromHole = 10;
loadCellDistanceFromShaft = 10;
loadCellHoleGap = 15;
loadCellHoleSize = 5;
loadCellHolePos1 = [-case.x/2,5,loadCell.z/2];
loadCellHolePos2 = [-case.x/2,5+loadCellHoleGap,loadCell.z/2];

lockingInsertSize = 3.5;
module motorShaft(){
    intersection(){
    translate([-motorShaftDim.x/2,-motorShaftDim.y/2,motorShaftBase.y]) cube(motorShaftDim);        
    translate([0,0,motorShaftBase.y]) cylinder(d=motorShaftBase.x,h=motorShaftDim.z,$fn=60);
    }
    cylinder(d=motorShaftBase.x,h=motorShaftBase.y,$fn=60);
}

module lockingScrewHole(){
    translate([0,0,motorShaftDim.z-lockingInsertSize/2]) rotate([90,0,0]) cylinder(d=lockingInsertSize,h=caseBackLengthFromHole,$fn=60);
}

module loadCellHoles(){
    translate(loadCellHolePos1) rotate([0,90,0]) cylinder(d=loadCellHoleSize,h=case.x,$fn=60);
    translate(loadCellHolePos2) rotate([0,90,0]) cylinder(d=loadCellHoleSize,h=case.x,$fn=60);
}

difference(){
    translate([-case.x/2,-caseBackLengthFromHole,0]) cube(case);
    motorShaft();
    lockingScrewHole();
    translate([-loadCell.x/2,loadCellDistanceFromShaft,0]) cube(loadCell);
    translate([0,loadCellDistanceFromShaft,0]) loadCellHoles();
}
