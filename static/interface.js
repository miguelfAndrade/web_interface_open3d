import * as THREE from 'three';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

const container = document.querySelector('#canvas');
const windowSizePercentage = 0.8;
let viewportWidth = window.innerWidth*windowSizePercentage;
let viewportHeight = window.innerHeight*windowSizePercentage;

let sendConfigButton = document.getElementById('sendConfig');

let divPoissonParams = document.getElementById('poissonParams');
let divAlphaParams = document.getElementById('alphaParams');

let alphaValueLabel = document.getElementById('alphaLabel');
let voxelLabel = document.getElementById('voxelLabel');
let pointNeighborsLabel = document.getElementById('pointNeighborsLabel');
let pointRadiusLabel = document.getElementById('pointRadiusLabel');
let poissonDepthLabel = document.getElementById('poissonDepthLabel');
// let poissonWidthLabel = document.getElementById('poissonWidthLabel');
// let poissonScaleLabel = document.getElementById('poissonScaleLabel');

let voxelSlider = document.getElementById('voxelSlider');
let methodDropdown = document.getElementById('method');
let alphaValue = document.getElementById('alphaValue');
let poissonDepthValue = document.getElementById('poissonDepth');
// let poissonWidthValue = document.getElementById('poissonWidth');
// let poissonScaleValue = document.getElementById('poissonScale');
let poissonLinearFitValue = document.getElementById('poissonLinearFit');
let pointNeighborsValue = document.getElementById('pointNeighborsValue');
let pointRadiusValue = document.getElementById('pointRadiusValue');

let methodType;
let voxelDownsampling;
let pointNeighbors;
let pointRadius;
let poissonDepth;
let poissonWidth;
let poissonScale;
let poissonLinearFit;
let alpha;

window.onload = function() {
	
	methodType = methodDropdown.value;
	voxelDownsampling = voxelSlider.value/100;
	pointNeighbors = pointNeighborsValue.value;
	pointRadius = pointRadiusValue.value/10;
	poissonDepth = poissonDepthValue.value;
	poissonWidth = 0;
	poissonScale = 0;
	poissonLinearFit = poissonLinearFitValue.checked.toString();
	alpha = alphaValue.value/100;
	
	
	alphaValueLabel.innerHTML = alpha;
	voxelLabel.innerHTML = voxelDownsampling;
	pointNeighborsLabel.innerHTML = pointNeighbors;
	pointRadiusLabel.innerHTML = pointRadius;
	poissonDepthLabel.innerHTML = poissonDepth;
}

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

// const material = new THREE.MeshBasicMaterial({color: 0x4283E5});
const directionalLight = new THREE.DirectionalLight(0xffffff, 1);

const controls = new OrbitControls( camera, renderer.domElement );
controls.target.set( 0, 0, 0 );
controls.update();
controls.enablePan = false;
controls.enableDamping = true;

scene.add( new THREE.AxesHelper() );
scene.add(directionalLight);

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
		// model.material = material;
		// console.log(model);
		// let mesh = new THREE.Mesh(model, material);
		// model = mesh;
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

methodDropdown.oninput = function () {
	switch (this.value) {
		case 'poisson':
			divPoissonParams.style.visibility = 'visible';
			divAlphaParams.style.visibility = 'hidden';
			break;
		case 'alpha':
			divPoissonParams.style.visibility = 'hidden';
			divAlphaParams.style.visibility = 'visible';
			break;
		case 'ball':
			divPoissonParams.style.visibility = 'hidden';
			divAlphaParams.style.visibility = 'hidden';
			break;
		default:
			break;
	}
	methodType = this.value;
}

pointNeighborsValue.oninput = function() {
	pointNeighborsLabel.innerHTML = this.value;
	pointNeighbors = this.value;
}

pointRadiusValue.oninput = function() {
	pointRadiusLabel.innerHTML = this.value/10;
	pointRadius = this.value/10;
}

voxelSlider.oninput = function() {
	voxelLabel.innerHTML = this.value/100;
	voxelDownsampling = this.value/100;
}

alphaValue.oninput = function() {
	alphaValueLabel.innerHTML = this.value/100;
	alpha = this.value/100;
}

poissonDepthValue.oninput = function() {
	poissonDepthLabel.innerHTML = this.value;
	poissonDepth = this.value;
}

poissonLinearFitValue.oninput = function() {
	poissonLinearFit = this.checked.toString();
}



sendConfigButton.onclick = function() {
	try {
		sendParametersData();
	}
	catch (error) {
		console.error( error );
	}
}

async function sendParametersData() {
	fetch("/parameters", {
	  method: "POST",
	  body: JSON.stringify({
		method: methodType,
		voxelDownsampling: voxelDownsampling,
		pointNeighbors: pointNeighbors,
		pointRadius: pointRadius,
		poissonDepth: poissonDepth,
		poissonWidth: poissonWidth,
		poissonScale: poissonScale,
		poissonLinearFit: poissonLinearFit,
		alpha: alpha
	  }),
	  headers: {
		"Content-type": "application/json; charset=UTF-8"
	  }
	})
	  .then((response) => response.json())
	  .then((json) => console.log(json));
}


