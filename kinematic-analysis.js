
KinematicAnalysisApp = function () {

  Sim.App.call(this);

}

KinematicAnalysisApp.prototype = new Sim.App();

KinematicAnalysisApp.prototype.init = function (params) {

  // Borrow Sim.App initializer code to set up scene, renderer, default camera
  Sim.App.prototype.init.call(this, params);

  // Create a directional light to show off the model
  var light = new THREE.DirectionalLight( 0xffffff, 1);
  light.position.set(0, -1, 1).normalize();
  this.scene.add(light);

  this.camera.position.set(0, 0, 5);

  // Let the floor grid add itself to the scene
  var floor = new Floor();
  floor.init({root: this});

  // Keep track of scene dragging
  this.lastX = 0;
  this.lastY = 0;
  this.mouseDown = false;
}


//
// A fixed grid floor
//

Floor = function ()
{
  return Sim.Object.call(this);
}
Floor.prototype = new Sim.Object();

Floor.prototype.init = function (params)
{
  var geometry = new THREE.PlaneGeometry(10, 10, 10, 10)
    , material = new THREE.MeshBasicMaterial({
        color: 0x010101,
        opacity: .2,
        transparent: true,
        wireframe: true,
        wireframeLinewidth: 2
    })
    , floor = new THREE.Mesh(geometry, material);
  params.root.scene.add(floor);
}
