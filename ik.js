const LENGTH_HUMERUS = 800;
const LENGTH_RADIUS = 800;


// based on the math here:
// http://math.stackexchange.com/a/1367732

// x1,y1 is the center of the first circle, with radius r1
// x2,y2 is the center of the second ricle, with radius r2
function intersectTwoCircles(x1, y1, r1, x2, y2, r2) {
  var centerdx = x1 - x2;
  var centerdy = y1 - y2;
  var R = Math.sqrt(centerdx * centerdx + centerdy * centerdy);
  if (!(Math.abs(r1 - r2) <= R && R <= r1 + r2)) { // no intersection
    return []; // empty list of results
  }
  // intersection(s) should exist

  var R2 = R * R;
  var R4 = R2 * R2;
  var a = (r1 * r1 - r2 * r2) / (2 * R2);
  var r2r2 = (r1 * r1 - r2 * r2);
  var c = Math.sqrt(2 * (r1 * r1 + r2 * r2) / R2 - (r2r2 * r2r2) / R4 - 1);

  var fx = (x1 + x2) / 2 + a * (x2 - x1);
  var gx = c * (y2 - y1) / 2;
  var ix1 = fx + gx;
  var ix2 = fx - gx;

  var fy = (y1 + y2) / 2 + a * (y2 - y1);
  var gy = c * (x1 - x2) / 2;
  var iy1 = fy + gy;
  var iy2 = fy - gy;

  // note if gy == 0 and gx == 0 then the circles are tangent and there is only one solution
  // but that one solution will just be duplicated as the code is currently written
  return [
    [ix1, iy1],
    [ix2, iy2]
  ];
}


class IKSolver {
  constructor(source, length1, length2) {
    this.source = source;
    this.r1 = length1;
    this.r2 = length2;
    this.joint = undefined;
    this.target = undefined;

    this.rootTheta = 0;
    this.jointTheta = 0;
  }

  _populatePtsOfIntersection(_target) {
    this.joint = intersectTwoCircles(
      this.source.x,
      this.source.y,
      this.r1 / 2,
      _target.x,
      _target.y,
      this.r2 / 2
    )[0] || this.joint || [0, 0];


    let m1 = ((this.target.y - this.joint[1]) /
      (this.target.x - this.joint[0]));
    let m2 = ((this.joint[1] - this.source.y) /
      (this.joint[0] - this.source.x));

    this.jointTheta = atan2(
      (m2 - m1), (1 + m1 * m2)
    );


    this.rootTheta = (PI) + atan2(
      (this.joint[1] - this.source.y),
      (this.joint[0] - this.source.x)
    );

    return this.joint;
  }

  solveLinePts(target) {
    // Return the line start and stop points
    // to touch the target

    this.target = target;
    let results = this._populatePtsOfIntersection(target);
    this.draw();

  }

  draw() {
    noFill();
    strokeWeight(2);
    stroke(200);
    // Draw a circle centered at target;
    circle(this.target.x, this.target.y, this.r2)
    // Draw a circle centered at source;
    circle(this.source.x, this.source.y, this.r1)

    fill(255, 0, 0);
    strokeWeight(0);
    let results = this.joint;
    if (results[0]) {
      // draw joint actuator
      circle(results[0], results[1], 20);

      // draw limbs:
      stroke(0);
      strokeWeight(4);
      line(this.source.x, this.source.y, results[0], results[1])
      line(results[0], results[1], this.target.x, this.target.y)


      // show angles:
      fill(255, 0, 255);
      noStroke();
      textSize(40);
      text(round(((180 * this.jointTheta / PI) - 720) % 180) + "º", this.joint[0] + 10, this.joint[1] + 10, 200);
      text(round(((180 * this.rootTheta / PI) - 720) % 180) + "º", this.source.x + 10, this.source.y - 50, 200)
    }

  }
}


let iks;
let currentPoint;
let targetPoint;
let newF = 0;

function setup() {
  createCanvas(1200, 600);
  iks = new IKSolver(createVector(width / 2, height), LENGTH_HUMERUS, LENGTH_RADIUS);
  currentPoint = createVector(width/2, height/2);
  targetPoint = createVector(width/2, height/2);
}

function mousePressed() {
  targetPoint = createVector(mouseX, mouseY);
  newF = frameCount;
}



function draw() {
  background(220);
  iks.solveLinePts(currentPoint);
  
  currentPoint = p5.Vector.lerp(currentPoint,targetPoint, (frameCount - newF)/100);

  // plot the telemetry:
  fill(255, 0, 255);
  noStroke();
  ellipse(targetPoint.x, targetPoint.y, 5);
}
