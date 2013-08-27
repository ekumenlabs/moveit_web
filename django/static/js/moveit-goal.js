// @constructor
//
// @param params - object with goal options:
// @param pose (optional) - ROSLIB.Pose object to call setPose() with.
// @param scene (optional) - THREE.Scene object (as in ``new ROS3D.Viewer().scene``) to add the goal to.
MoveItGoal = function (params, pose, scene) {
  params = params || {};

  // contruct the small dot to show the position
  var dot_mesh = new THREE.Mesh(
    MoveItGoal.dotGeometry,
    MoveItGoal.statusMaterials.unknown
  );
  this.add(dot_mesh);

  // construct the axes
  var axes = new ROS3D.Axes();
  this.add(axes);

  // TODO: construct the direction arrow

  // add this goal to the general repository
  MoveItGoal.allGoals.push(this);

  // set position and orientation, if given
  pose && this.setPose(pose);

  // add to the scene, if given
  scene && scene.add(this);
}

// One repository for all goals created.
//
// TODO: allow to remove this goals from the scene and deallocate them from
// memory as a one "clearAllGoals()" call.
MoveItGoal.allGoals = [];

MoveItGoal.dotGeometry = new THREE.SphereGeometry(0.03,0.03,0.03);

MoveItGoal.statusMaterials = {
  unknown: new THREE.MeshBasicMaterial({ color: 0xffffff }),
  reachable: new THREE.MeshBasicMaterial({ color: 0x00ff00 }),
  unreachable: new THREE.MeshBasicMaterial({ color: 0xff0000 })
};

// Returns a reference to the last goal created, null if there's none.
MoveItGoal.latest = function () {
  if (this.allGoals.length > 0)
    return this.allGoals[this.allGoals.length - 1];
  return null;
}

MoveItGoal.prototype = new THREE.Object3D();

// Set both position and orientation (using a quaternion).
MoveItGoal.prototype.setPose = function (pose) {
  this.setPosition(pose.position);
  this.setOrientation(pose.orientation);
}

MoveItGoal.prototype.setPosition = function (position) {
  // NOTE that a ROSLIB.Vector3 is not a THREE.Vector3... it doesn't even share
  // a prototype, _as it should_. Check out the code and see.
  this.position.x = position.x;
  this.position.y = position.y;
  this.position.z = position.z;
}

MoveItGoal.prototype.setOrientation = function (postion) {
}
