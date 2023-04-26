import * as THREE from 'three';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

const container = document.querySelector('#canvas');
const windowSizePercentage = 0.8;
let viewportWidth = window.innerWidth*windowSizePercentage;
let viewportHeight = window.innerHeight*windowSizePercentage;

let voxelSlider = document.getElementById('voxelSlider');
let method = document.getElementById('method');
let divPoissonParams = document.getElementById('poissonParams');

const renderer = new THREE.WebGLRenderer( { antialias: true } );
renderer.setPixelRatio( window.devicePixelRatio );
renderer.setSize( viewportWidth, viewportHeight );
renderer.outputEncoding = THREE.sRGBEncoding;
container.appendChild( renderer.domElement );

const fov = 75;
const aspect = 2; 
const near = 0.1;
const far = 50;
const camera = new THREE.PerspectiveCamera(fov, aspect, near, far);
camera.position.z = 3;

const scene = new THREE.Scene();
// scene.background = new THREE.Color( 0xbfe3dd );
scene.background = new THREE.Color( 0xe2e2e2 );

const controls = new OrbitControls( camera, renderer.domElement );
controls.target.set( 0, 0, 0 );
controls.update();
controls.enablePan = false;
controls.enableDamping = true;

scene.add( new THREE.AxesHelper() );

// load a resource
const loader = new GLTFLoader();
let model = new THREE.Object3D();


animate();


window.onresize = function () {

	let newWidth = window.innerWidth*windowSizePercentage;
	let newHeight = window.innerHeight*windowSizePercentage;

	camera.aspect = newWidth / newHeight;
	camera.updateProjectionMatrix();

	renderer.setSize( newWidth, newHeight );

};

function objLoader() {
	loader.load( 'model', function ( gltf ) {
		scene.remove(model);
		
		model = gltf.scene;
		model.rotation.x = -Math.PI/2;
		scene.add( model );
	
		// animate();
	
		}, undefined, function ( error ) {
	
			console.error( error );
	
		} 
	);
}

function animate() {

	requestAnimationFrame( animate );
	
	objLoader();

	controls.update();

	renderer.render( scene, camera );

}

voxelSlider.oninput = function() {
	try {
		sendParametersData(method.value, this.value, 7);
	}
	catch (error) {
		console.error( error );
	}
};


method.oninput = function () {
	if(this.value == 'poisson') {
		divPoissonParams.style.visibility = 'visible';
	}
	else {
		divPoissonParams.style.visibility = 'hidden';
	}
	console.log(voxelSlider.value);
	try {
		sendParametersData(this.value, voxelSlider.value, 7);
	}
	catch (error) {
		console.error( error );
	}
}

async function sendParametersData(method, voxValue, defValue) {
	fetch("/parameters", {
	  method: "POST",
	  body: JSON.stringify({
		method: method,
		voxelValue: voxValue,
		point_neighbors: 0,
		point_radius: 0.1,
		poisson_depth: 5,
		poisson_width: 0,
		poisson_scale: 1.1,
		poisson_linear_fit: false
	  }),
	  headers: {
		"Content-type": "application/json; charset=UTF-8"
	  }
	})
	  .then((response) => response.json())
	  .then((json) => console.log(json));
}

