import * as THREE from 'three';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

const container = document.querySelector('#c');
const windowSizePercentage = 0.8;

const renderer = new THREE.WebGLRenderer( { antialias: true } );
renderer.setPixelRatio( window.devicePixelRatio );
renderer.setSize( window.innerWidth*windowSizePercentage, window.innerHeight*windowSizePercentage );
renderer.outputEncoding = THREE.sRGBEncoding;
container.appendChild( renderer.domElement );

const fov = 75;
const aspect = 2; 
const near = 0.1;
const far = 50;
const camera = new THREE.PerspectiveCamera(fov, aspect, near, far);
camera.position.z = 3;

const scene = new THREE.Scene();
scene.background = new THREE.Color( 0xbfe3dd );

const controls = new OrbitControls( camera, renderer.domElement );
controls.target.set( 0, 0, 0 );
controls.update();
controls.enablePan = false;
controls.enableDamping = true;

// load a resource
const loader = new GLTFLoader();

loader.load( 'model', function ( gltf ) {

	const model = gltf.scene;
	model.rotation.x = -Math.PI/2;
	scene.add( model );

	animate();

	}, undefined, function ( error ) {

		console.error( error );

	} 
);

window.onresize = function () {

	let newWidth = window.innerWidth*windowSizePercentage;
	let newHeight = window.innerHeight*windowSizePercentage;

	camera.aspect = newWidth / newHeight;
	camera.updateProjectionMatrix();

	renderer.setSize( newWidth, newHeight );

};

function animate() {

	requestAnimationFrame( animate );

	controls.update();

	renderer.render( scene, camera );

}
