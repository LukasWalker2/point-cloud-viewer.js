//TODO:COLOR Scale
//gridline algorithm source https://stackoverflow.com/questions/361681/algorithm-for-nice-grid-line-intervals-on-a-graph -Adam Liss
function calculateGridLines(range , subdivisions) {
    // calculate an initial guess at step size
    const tempStep = range / subdivisions;

    // get the magnitude of the step size
    const mag = Math.floor(Math.log10(tempStep));
    const magPow = Math.pow(10, mag);

    // calculate most significant digit of the new step size
    const magMsd = Math.round(tempStep / magPow + 0.5);

    // promote the MSD to either 1, 2, or 5
    let resultMsd;
    if (magMsd > 5) {
        resultMsd = 10;
    } else if (magMsd > 2) {
        resultMsd = 5;
    } else if (magMsd > 1) {
        resultMsd = 2;
    } else {
        resultMsd = 1;
    }

    return resultMsd * magPow;
}






class PointCloud3D {
  constructor(wrapperId, width, height, points, color_callback = ()=> 0x03fcf8) {
    this.scene = new THREE.Scene();
    this.range = this.findMinMaxValues(points)
    this.getColor = (max, min, color) => {const r = color_callback(max, min, color); return `rgb(${Math.floor(r[0])},${Math.floor(r[1])},${Math.floor(r[2])})`}

    this.aspect_ratio  = width/height;

    //the space limit values are normalized to in the image space 10 means the the longest axis of the pointcloud has a length of 10 in the image space 
    this.scale_limit = 10;

    //settings otho mode = ortho camera otherwise a perspective camera will be used square mode means all axes have the length of scale limit in the image space 
    this.square_mode = true;
    this.ortho_mode = false;


    // factor that determines size of the point in the pointcloud 
    this.point_scale = 1;

    this.renderer = new THREE.WebGLRenderer({ alpha: true });

    this.wrapper = document.getElementById(wrapperId);
    this.width = width;
    this.height = height;
    this.points = points;


    //vectors used to store x y movement for rotation 2d because the pointcloud is only rotated around 2 axes
    this.rotateStart = new THREE.Vector2();
    this.rotateEnd = new THREE.Vector2();
    this.rotateDelta = new THREE.Vector2();
    this.rotatationState = new THREE.Vector2();

    this.isMouseDown = false;

    this.offsetX = -width / 2;
    this.offsetY = -height / 2;

    this.point_limits = this.findMinMaxDimensions(points)

    //an offset space between axis and points for better visual appearence 
    var zoom = 2
    //a faktor that determines how much the pointcloud viewer is shifted downwards based on it's parent element's dimensions
    this.downshift = 0.25;

    //var that store settings 
    this.interpolation_plane = false;
    this.remove_points = true;
    this.displacement = 0;

    //vars that are important for normalisations and translation to image space 
    this.point_limits.min.x = Math.floor(this.point_limits.min.x-zoom)
    this.point_limits.min.y = Math.floor(this.point_limits.min.y-zoom)
    this.point_limits.min.z = Math.floor(this.point_limits.min.z-zoom)

    this.point_limits.max.x = Math.ceil(this.point_limits.max.x+zoom)
    this.point_limits.max.y = Math.ceil(this.point_limits.max.y+zoom)
    this.point_limits.max.z = Math.ceil(this.point_limits.max.z+zoom)


    this.x_range = this.point_limits.max.x - this.point_limits.min.x;
    this.y_range = this.point_limits.max.y - this.point_limits.min.y;
    this.z_range = this.point_limits.max.z - this.point_limits.min.z;

    this.longest_axis = Math.max(this.x_range, this.y_range, this.z_range)
    this.scale = this.scale_limit/this.longest_axis;
    this.diagonal = Math.sqrt(this.x_range * this.x_range + this.y_range * this.y_range + this.z_range * this.z_range);

    this.camera = this.ortho_mode ? new THREE.OrthographicCamera(this.translateToScale(-this.aspect_ratio * this.longest_axis),this.translateToScale(this.aspect_ratio * this.longest_axis), this.translateToScale(this.longest_axis), this.translateToScale(-this.longest_axis), 0, 1000) : new THREE.PerspectiveCamera(75, width / height, 0.1, 200);
    this.distance_const = 10;

    // const ambientLight = new THREE.AmbientLight(0xffffff, 0.0);
    // this.scene.add(ambientLight);

    //Sequential invocation of internal functions to create a basic point cloud
    this.init();
    this.createPointCloud(this.points);
    this.createAxes();
    this.createRaster(5);
    this.initPosBase();
    this.shitftDownBase();
    this.createSettings();
    this.animate();
  }

  //sets basic parameters to create the pointcloud which only have to be done once 
  init() {
    this.color_scale =  this.drawColorScale([this.range.minVal,this.range.maxVal], 'y', this.height*0.6, 40, calculateGridLines)
    this.renderer.setSize(this.width, this.height);
    this.wrapper.style.display = 'inline-block'
    this.color_scale.style.display = 'inline-block'
    this.color_scale.style.marginTop = `${this.height*0.2}px`
    this.color_scale.style.marginBottom = `${this.height*0.2}px`
    this.renderer.domElement.style.display = 'inline-block'
    this.wrapper.appendChild(this.color_scale)
    this.wrapper.appendChild(this.renderer.domElement);


    this.camera.position.z = this.translateToScale(this.diagonal) + this.distance_const;
    this.sphere_radius = this.translateToScale(this.diagonal) / 100

    window.addEventListener('resize', () => this.onWindowResize());
    document.addEventListener('mouseup', () => this.onMouseUp(), false);
    this.renderer.domElement.addEventListener('mousedown', (event) => this.onMouseDown(event), false);
    this.renderer.domElement.addEventListener('mousemove', (event) => this.onMouseMove(event), false);
    this.renderer.domElement.classList.add('mobile_event_translation')
  }

  initPosBase(){
      this.pointCloud.rotation.x += 0.08 * Math.PI;
      this.pointCloud.rotation.y -= 0.25 * Math.PI;
      this.axesGroup.rotation.x += 0.08 * Math.PI;
      this.axesGroup.rotation.y  -= 0.25 * Math.PI;
      this.grid_group.rotation.x += 0.08 *  Math.PI;
      this.grid_group.rotation.y -= 0.25 * Math.PI;
  }

  //shifts down the basic elements of the pointcloud, beacause i think it looks better this way 
  shitftDownBase(){
    
    this.grid_group.position.y -= this.translateToScale(this.longest_axis)*this.downshift;
    this.pointCloud.position.y -= this.translateToScale(this.longest_axis)*this.downshift;
    this.axesGroup.position.y -= this.translateToScale(this.longest_axis)*this.downshift;
  }

  initPos(target){
      target.rotation.x = this.axesGroup.rotation.x; 
      target.rotation.y = this.axesGroup.rotation.y;
  }

  shiftDown(target){
    target.position.y -= this.translateToScale(this.longest_axis)*this.downshift;
  }

  translateToScale(val){
    return this.scale * val;
  }


  //creates the axes of the pointcloud 
  createAxes() {
    this.axesGroup = new THREE.Group();
    const lineMaterial = new THREE.LineBasicMaterial({ color: 0x000000 });

    const xAxisGeometry = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(0, 0, 0), new THREE.Vector3(this.translateToScale(this.square_mode ? this.longest_axis : this.x_range), 0, 0)]);
    const xAxis = new THREE.Line(xAxisGeometry, lineMaterial);
    this.axesGroup.add(xAxis);

    const yAxisGeometry = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(0, 0, 0), new THREE.Vector3(0, this.translateToScale(this.square_mode ? this.longest_axis : this.y_range), 0)]);
    const yAxis = new THREE.Line(yAxisGeometry, lineMaterial);
    this.axesGroup.add(yAxis);

    const zAxisGeometry = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(0, 0, 0), new THREE.Vector3(0, 0, this.translateToScale(this.square_mode ? this.longest_axis : this.z_range))]);
    const zAxis = new THREE.Line(zAxisGeometry, lineMaterial);
    this.axesGroup.add(zAxis);

    this.scene.add(this.axesGroup);
  }


  //creates grid with labels for the pointcloud 
  createRaster(res) {
    const yoffset = 0.3;
    this.grid_group = new THREE.Group();

    const min_axe_range = Math.floor(Math.min(this.x_range, this.y_range, this.z_range));

    //const step_size = min_axe_range / res;

    const lineMaterial = new THREE.LineBasicMaterial({ color: 0xbfbebb });
    const textMaterial = new THREE.MeshBasicMaterial({ color: 0x000000 });

    const xStep = calculateGridLines(this.square_mode ? this.longest_axis : this.x_range, res);
    const yStep = calculateGridLines(this.square_mode ? this.longest_axis : this.y_range,res);
    const zStep = calculateGridLines(this.square_mode ? this.longest_axis : this.z_range, res);

  

    for (var i = 0; i < (this.square_mode ? this.longest_axis : this.x_range); i += xStep) {
      if (i == 0) continue;
      (function (i, self) {
        const xAxisGeometry1 = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(self.translateToScale(i), 0, 0), new THREE.Vector3(self.translateToScale(i), 0, self.translateToScale(self.square_mode ? self.longest_axis : self.z_range))]);
        const xAxisGeometry2 = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(self.translateToScale(i), 0, 0), new THREE.Vector3(self.translateToScale(i), self.translateToScale(self.square_mode ? self.longest_axis : self.y_range, 0))]);
        const l1 = new THREE.Line(xAxisGeometry1, lineMaterial);
        const l2 = new THREE.Line(xAxisGeometry2, lineMaterial);
        self.grid_group.add(l1)
        self.grid_group.add(l2)


        const loader = new THREE.FontLoader();
        loader.load('/font.json', function (font) {

          let textFont = font;
          const textGeometry = new THREE.TextGeometry((i + self.point_limits.min.x).toFixed(1).toString(), {
            font: textFont,
            size: 0.25, 
            height: 0.001, 
          });
          const textMesh = new THREE.Mesh(textGeometry, textMaterial);
          textMesh.position.set(self.translateToScale(i), 0-yoffset, self.translateToScale(self.square_mode ? self.longest_axis + 0.9 : self.z_range + 0.9)); 
          textMesh.rotation.set(0, 0, 0);
          textMesh.rotateY(-self.grid_group.rotation.y);
          textMesh.rotateX(-self.grid_group.rotation.x);
          self.grid_group.add(textMesh);
        }
        )
      })(i, this);


    }

    for (var i = 0; i < (this.square_mode ? this.longest_axis : this.y_range); i += yStep) {
      if (i == 0) continue;
      (function (i, self) {
      const yAxisGeometry1 = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(0, self.translateToScale(i), 0), new THREE.Vector3(0, self.translateToScale(i), self.translateToScale(self.square_mode ? self.longest_axis : self.z_range))]);
      const yAxisGeometry2 = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(0, self.translateToScale(i), 0), new THREE.Vector3(self.translateToScale(self.square_mode ? self.longest_axis : self.x_range), self.translateToScale(i), 0)]);
      const l1 = new THREE.Line(yAxisGeometry1, lineMaterial);
      const l2 = new THREE.Line(yAxisGeometry2, lineMaterial);
      self.grid_group.add(l1)
      self.grid_group.add(l2)
      const loader = new THREE.FontLoader();
      loader.load('/font.json', function (font) {
        let textFont = font;
        const textGeometry = new THREE.TextGeometry((self.point_limits.min.y + i).toFixed(1).toString(), {
          font: textFont,
          size: 0.25,
          height: 0.001, 
        });
        const textMesh = new THREE.Mesh(textGeometry, textMaterial);
        textMesh.position.set(self.translateToScale(self.square_mode ? self.longest_axis + 0.9 : self.x_range +0.9),self.translateToScale(i), 0); 
        textMesh.rotation.set(0, 0, 0);
        textMesh.rotateY(-self.grid_group.rotation.y);
        textMesh.rotateX(-self.grid_group.rotation.x);
        self.grid_group.add(textMesh);
      }
      )
    })(i, this);

    }


    for (var i = 0; i <  (this.square_mode ? this.longest_axis : this.z_range); i += zStep) {
      if (i == 0) continue;
      (function (i, self) {
      const zAxisGeometry1 = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(0, 0, self.translateToScale(i)), new THREE.Vector3(0, self.translateToScale(self.square_mode ? self.longest_axis : self.y_range), self.translateToScale(i))]);
      const zAxisGeometry2 = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(0, 0, self.translateToScale(i)), new THREE.Vector3(self.translateToScale(self.square_mode ? self.longest_axis : self.x_range), 0, self.translateToScale(i))]);
      const l1 = new THREE.Line(zAxisGeometry1, lineMaterial);
      const l2 = new THREE.Line(zAxisGeometry2, lineMaterial);
      self.grid_group.add(l1)
      self.grid_group.add(l2)
      const loader = new THREE.FontLoader();
      loader.load('/font.json', function (font) {

        let textFont = font;
        const textGeometry = new THREE.TextGeometry((self.point_limits.min.z + i).toFixed(1).toString(), {
          font: textFont,
          size: 0.25, 
          height: 0.001, 
        });
        const textMesh = new THREE.Mesh(textGeometry, textMaterial);
        textMesh.position.set(self.translateToScale(self.square_mode ? self.longest_axis + 0.9 : self.x_range +0.9), 0-yoffset, self.translateToScale(i));
        textMesh.rotateY(-self.grid_group.rotation.y);
        textMesh.rotateX(-self.grid_group.rotation.x);
        self.grid_group.add(textMesh);
      }
      )
    })(i, this);

      (function (self){
        const loader = new THREE.FontLoader();
      loader.load('/font.json', function (font) {

        let textFont = font;
        let settings = {
          font: textFont,
          size: 0.3, 
          height: 0.001, 
        }
        const z_axis = new THREE.TextGeometry('Z', settings);
        const x_axis = new THREE.TextGeometry('X', settings);
        const y_axis = new THREE.TextGeometry('Y', settings);
        const z_mesh = new THREE.Mesh(z_axis, textMaterial);
        const x_mesh = new THREE.Mesh(x_axis, textMaterial);
        const y_mesh = new THREE.Mesh(y_axis, textMaterial);
        z_mesh.position.set(0, 0, self.translateToScale(self.square_mode ? self.longest_axis + 1.8 : self.z_range + 1.8)); 
        z_mesh.rotation.set(0, 0, 0);
        z_mesh.rotateY(-self.grid_group.rotation.y);
        z_mesh.rotateX(-self.grid_group.rotation.x);
        x_mesh.position.set(self.translateToScale(self.square_mode ? self.longest_axis + 1.8 : self.x_range + 1.8), 0, 0); 
        x_mesh.rotation.set(0, 0, 0);
        x_mesh.rotateY(-self.grid_group.rotation.y);
        x_mesh.rotateX(-self.grid_group.rotation.x);
        y_mesh.position.set(0, self.translateToScale(self.square_mode ? self.longest_axis + 0.5 : self.y_range + 0.5) ,0); 
        y_mesh.rotation.set(0, 0, 0);
        y_mesh.rotateY(-self.grid_group.rotation.y);
        y_mesh.rotateX(-self.grid_group.rotation.x);
        self.grid_group.add(z_mesh);
        self.grid_group.add(x_mesh);
        self.grid_group.add(y_mesh);
      }
      )
        
      })
      (this);

    }



    this.scene.add(this.grid_group)




  }


  createPointCloud(points) {
    this.pointCloud = new THREE.Group();

    const geometry = new THREE.SphereGeometry(this.sphere_radius, 64, 64);
    points.forEach(point => {
      let material = new THREE.MeshBasicMaterial({ color: 0x03fcf8 });
      if(point.val && this.range.minVal && this.range.maxVal){
          material = new THREE.MeshBasicMaterial({ color:  new THREE.Color(this.getColor(this.range.maxVal,this.range.minVal, point.val))});
      }
      const sphere = new THREE.Mesh(geometry, material)
      sphere.position.x = this.translateToScale(point.x - this.point_limits.min.x);
      sphere.position.y = this.translateToScale(point.y - this.point_limits.min.y);
      sphere.position.z = this.translateToScale(point.z - this.point_limits.min.z);
      this.pointCloud.add(sphere)
    });
    this.scene.add(this.pointCloud)
  }

  //eventhandler und functions to move the pointcloud
  animate() {
    const animate = () => {
      requestAnimationFrame(animate);
      this.renderer.render(this.scene, this.camera);
    };
    animate();
  }

  onWindowResize() {
    this.camera.aspect = this.width / this.height;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(this.width, this.height);
  }

  onMouseDown(event) {
    this.isMouseDown = true;
    this.rotateStart.set(event.clientX, event.clientY);
  }

  onMouseMove(event) {
    if (!this.isMouseDown) return;

    this.rotateEnd.set(event.clientX, event.clientY);
    this.rotateDelta.subVectors(this.rotateEnd, this.rotateStart);

    const sensitivity = 0.01;

    this.pointCloud.rotation.x += this.rotateDelta.y * sensitivity;
    this.pointCloud.rotation.y += this.rotateDelta.x * sensitivity;
    this.axesGroup.rotation.x += this.rotateDelta.y * sensitivity;
    this.axesGroup.rotation.y += this.rotateDelta.x * sensitivity;
    this.grid_group.rotation.x += this.rotateDelta.y * sensitivity;
    this.grid_group.rotation.y += this.rotateDelta.x * sensitivity;
    if(this.planeGroup){
      this.planeGroup.rotation.x += this.rotateDelta.y * sensitivity;
      this.planeGroup.rotation.y += this.rotateDelta.x * sensitivity;
    }

    this.grid_group.children.forEach((child) => {
      if (child instanceof THREE.Mesh) {
        child.rotation.set(0, 0, 0);
        child.rotateY(-this.grid_group.rotation.y);
        child.rotateX(-this.grid_group.rotation.x); 
      }
    });
    this.rotateStart.copy(this.rotateEnd);
  }

  onMouseUp() {
    this.isMouseDown = false;
  }

  //same as dindMinMaxValues just for a number of lists determined by dict key 
  findMinMaxDimensions(points) {
    if (points.length == 0) {
      return { min: { x: 0, y: 0, z: 0 }, max: { x: 0, y: 0, z: 0 } };
    }

    const dimensions = Object.keys(points[0]);
    const initialMin = { ...points[0] };
    const initialMax = { ...points[0] };

    for (const point of points) {
      for (const dimension of dimensions) {
        initialMin[dimension] = Math.min(initialMin[dimension], point[dimension]);
        initialMax[dimension] = Math.max(initialMax[dimension], point[dimension]);
      }
    }

    return { min: initialMin, max: initialMax };
  }


  //self explanatory filters value until only max and min values left and returns those
  findMinMaxValues(data) {
    if (data.length == 0) {
      return { minVal: undefined, maxVal: undefined };
    }
    return data.reduce((result, obj) => {
      const value = obj.val || 0;
      result.minVal = Math.min(result.minVal !== undefined ? result.minVal : value, value);
      result.maxVal = Math.max(result.maxVal !== undefined ? result.maxVal : value, value);
      return result;
    }, { minVal: undefined, maxVal: undefined });
  }


  //creates the gear as settings button and the window using the module window-engine 
  //the gear is composed of svg elements added via js DOM api 
  createSettings(){
    var settings = false
    const dimension = 30
    const color = '#3d3d3d'
    var generic_svg = document.createElementNS('http://www.w3.org/2000/svg','svg')
    generic_svg.style.height = `${dimension}px`
    generic_svg.style.width = `${dimension}px`
    generic_svg.style.position = 'absolute'
    generic_svg.style.top =  '10px'
    generic_svg.style.right = '10px'
    generic_svg.style.transformOrigin = `50% 50%`
    generic_svg.style.transition = 'transform 1.5s ease-in-out'
    //generic_svg.style.transform = 'rotate(10deg)'

    var rotation = 0;
    generic_svg.addEventListener('click', (e) => 
      {
        generic_svg.style.transform = `rotate(${rotation+=180}deg)`
        setTimeout(()=>{
        if(!settings){
          var setting_window = new Window(this.wrapper.clientHeight * 0.3 > 300 ? this.wrapper.clientHeight * 0.3 : 300, this.wrapper.clientWidth*0.5 > 300 ? this.wrapper.clientWidth*0.5: 300, screen.width/2-(this.wrapper.clientWidth * 0.15),screen.height/2-(this.wrapper.clientHeight * 0.25))
          setting_window.frame.classList.add('mobile_event_translation')
          setting_window.frame.style.zIndex = 1000;


          const pointSizeDiv = document.createElement('div');
          pointSizeDiv.style.marginTop = '50px';
          pointSizeDiv.style.paddingLeft = '10px';
          
          const pointSizeLabel = document.createElement('label');
          pointSizeLabel.style.userSelect = 'none'
          pointSizeLabel.textContent = 'Point Size:';
          pointSizeLabel.style.fontFamily = 'Arial'
          pointSizeDiv.appendChild(pointSizeLabel);
      
          const pointSizeSlider = document.createElement('input');
          pointSizeSlider.type = 'range';
          pointSizeSlider.min = '0.1';
          pointSizeSlider.max = '2.0';
          pointSizeSlider.step = '0.1';
          pointSizeSlider.value = this.point_scale;
          pointSizeSlider.addEventListener('input', (event) => this.onPointSizeChange(event));
          pointSizeSlider.addEventListener('click', (event) => event.stopPropagation());
          pointSizeSlider.addEventListener('mousedown', (event) => event.stopPropagation());
          pointSizeSlider.addEventListener('mouseup', (event) => event.stopPropagation());
          pointSizeDiv.appendChild(pointSizeSlider);
      
          setting_window.frame.appendChild(pointSizeDiv);
      
          const ortho_modeDiv = document.createElement('div');
          ortho_modeDiv.style.marginTop = '10px';
          ortho_modeDiv.style.paddingLeft = '10px';
      
          const ortho_modeLabel = document.createElement('label');
          ortho_modeLabel.textContent = 'Ortho Mode:';
          ortho_modeLabel.style.userSelect = 'none'
          ortho_modeLabel.style.fontFamily = 'Arial'
          ortho_modeDiv.appendChild(ortho_modeLabel);
      
          const orthoSwitch = document.createElement('input');
          orthoSwitch.type = 'checkbox';
          orthoSwitch.checked = this.ortho_mode;
          orthoSwitch.addEventListener('change', () => this.onortho_modeChange());
          ortho_modeDiv.appendChild(orthoSwitch);
      
          setting_window.frame.appendChild(ortho_modeDiv);
      
          const square_modeDiv = document.createElement('div');
          square_modeDiv.style.marginTop = '10px';
          square_modeDiv.style.paddingLeft = '10px';
      
          const square_modeLabel = document.createElement('label');
          square_modeLabel.textContent = 'Square Mode:';
          square_modeLabel.style.userSelect = 'none'
          square_modeLabel.style.fontFamily = 'Arial'
          square_modeDiv.appendChild(square_modeLabel);
      
          const square_modeSwitch = document.createElement('input');
          square_modeSwitch.type = 'checkbox';
          square_modeSwitch.checked = this.square_mode;
          square_modeSwitch.addEventListener('change', () => this.onsquare_modeChange());
          square_modeDiv.appendChild(square_modeSwitch);
          setting_window.frame.appendChild(square_modeDiv);

          const plane_switch = document.createElement('input');
          plane_switch.type = 'checkbox';
          plane_switch.checked = this.interpolation_plane;
          plane_switch.addEventListener('change', () => this.on_interpolationPlaneModeChange());
          setting_window.frame.appendChild(plane_switch);

          const axisDropdown = document.createElement("select");
          axisDropdown.style.marginTop = "10px";
          axisDropdown.style.marginBottom = "10px";
          axisDropdown.style.userSelect = "none";
          axisDropdown.addEventListener("change", (event) => this.onAxisChange(event));
          setting_window.frame.appendChild(axisDropdown);

          const optionZ = document.createElement("option");
          optionZ.setAttribute("value","Z");
          optionZ.innerHTML = "Z";
          optionZ.style.userSelect = 'none'
  
          axisDropdown.appendChild(optionZ)

          const displacementSlider = document.createElement("input");
          displacementSlider.type = "range";
          displacementSlider.min = "0";
          displacementSlider.max = "10";
          displacementSlider.step = "0.1";
          displacementSlider.value = `${this.displacement}`;
          displacementSlider.style.marginTop = "20px";
          displacementSlider.addEventListener("click", (event) => event.stopPropagation());
          displacementSlider.addEventListener("mousedown", (event) => event.stopPropagation());
          displacementSlider.addEventListener("mouseup", (event) => event.stopPropagation());
          setting_window.frame.appendChild(displacementSlider);
          
          const displacementLabel = document.createElement('label'); 
          displacementLabel.style.marginLeft = '6px'
          displacementLabel.style.userSelect = 'none'
          displacementLabel.innerHTML = `${this.displacement.toFixed(1)}`

          setting_window.frame.appendChild(displacementLabel);
          displacementSlider.addEventListener("input", (event) => this.onDisplacementChange(event,displacementLabel)); 

          const plane_visability = document.createElement('input');
          plane_visability.type = 'checkbox';
          plane_visability.checked = this.interpolation_plane;
          plane_visability.addEventListener('change', () => {this.remove_points = !this.remove_points; if(!this.remove_points){this.scene.remove(this.pointCloud);this.createPointCloud(this.points);this.shiftDown(this.pointCloud);this.initPos(this.pointCloud)};this.updatePlane()});
          setting_window.frame.appendChild(plane_visability);
          
          setting_window.setCloseHook(()=>{settings=false})
          settings = true
        }
      }
    ,300)
    });

    
    var rect = document.createElementNS('http://www.w3.org/2000/svg','rect')
    rect.setAttribute('x',`${dimension/2 - dimension/12}`)
    rect.setAttribute('y','0')
    rect.setAttribute('rx',`${dimension/15}`)
    rect.setAttribute('ry',`${dimension/15}`)
    rect.style.height = `${dimension/6}px`
    rect.style.width = `${dimension/6}px`
    rect.style.fill = color


    for(var i = 0; i < 360; i+= 45){
      var rect2 = document.createElementNS('http://www.w3.org/2000/svg','rect')
      rect2.setAttribute('x',`${dimension/2 - dimension/12}`)
      rect2.setAttribute('y','0')
      rect2.setAttribute('rx',`${dimension/15}`)
      rect2.setAttribute('ry',`${dimension/15}`)
      rect2.style.height = `${dimension/6}px`
      rect2.style.width = `${dimension/6}px`
      rect2.style.fill = color
      rect2.style.transformOrigin = `50% ${dimension/2}px`
      rect2.style.transform = `rotate(${i}deg)`
      generic_svg.append(rect2)
    }


    var circle = document.createElementNS('http://www.w3.org/2000/svg','circle')
    circle.setAttribute('cx',`${dimension/2}`)
    circle.setAttribute('cy',`${dimension/2}`)
    circle.setAttribute('r',`${dimension/3-1}`)
    circle.style.fill = 'white'
    circle.style.stroke = color
    circle.style.strokeWidth = `${dimension/6}`


   
     
    this.wrapper.append(generic_svg)
    generic_svg.append(rect)
    generic_svg.append(circle)
  }

  onPointSizeChange(event) {
    this.point_scale = parseFloat(event.target.value);
    this.updatePointCloudPointSize();
  }

  updatePointCloudPointSize() {
    const geometry = new THREE.SphereGeometry(this.sphere_radius * this.point_scale, 64, 64);
    this.pointCloud.children.forEach((point) => {
      point.geometry = geometry;
    });
  }

  //functions that recomposes the pointcloudviewer respective to the change setting (event handler on setting change)
  onortho_modeChange() {
    this.ortho_mode = !this.ortho_mode;
    this.updateCamera();
    this.axesGroup.clear();
    this.createAxes();
    this.grid_group.clear();
    this.createRaster(5);
    this.pointCloud.clear();
    this.createPointCloud(this.points);
    this.initPosBase();
    this.shitftDownBase();
    this.updatePlane();
  }

  onsquare_modeChange() {
    this.square_mode = !this.square_mode;
    this.axesGroup.clear();
    this.createAxes();
    this.grid_group.clear();
    this.createRaster(5);
    this.pointCloud.clear();
    this.createPointCloud(this.points);
    this.initPosBase();
    this.shitftDownBase();
    this.updatePlane();
  }

  updateCamera() {
    this.scene.remove(this.camera)
    this.camera = this.ortho_mode ? new THREE.OrthographicCamera(this.translateToScale(-this.aspect_ratio * this.longest_axis),this.translateToScale(this.aspect_ratio * this.longest_axis), this.translateToScale(this.longest_axis), this.translateToScale(-this.longest_axis), 0, 1000) : new THREE.PerspectiveCamera(75, width / height, 0.1, 200);
    this.scene.add(this.camera);
    this.camera.position.z = this.translateToScale(this.diagonal) + this.distance_const;
  }

  //not used but created for update potential
  onAxisChange(event) {
    this.selectedAxis = event.target.value;
    this.updatePlane();
}

onDisplacementChange(event,label) {
    this.displacement = parseFloat(event.target.value);
    label.innerHTML = `${this.displacement.toFixed(1)}`
    this.updatePlane();
}

on_interpolationPlaneModeChange(){
  this.interpolation_plane = !this.interpolation_plane;
  if(!this.interpolation_plane && this.planeGroup){
    this.scene.remove(this.planeGroup);
    this.scene.remove(this.pointCloud);
    this.createPointCloud(this.points);
    this.shiftDown(this.pointCloud);
    this.initPos(this.pointCloud);
  }
  this.updatePlane()
}

//missleading function name returns the interpolated value of a new paint based on it's relative position to the existing point and their value
distanceToColor(point) {
  const calculateDistance = (p1, p2) => Math.sqrt(
      Math.pow(p2.x - p1.x, 2) +
      Math.pow(p2.y - p1.y, 2) +
      Math.pow(p2.z - p1.z, 2)
  );
  
  let totalweightedVal = 0;
  let totalWeight = 0;
  
  const len_z = this.translateToScale(this.square_mode ? this.z_range/this.longest_axis * this.longest_axis : this.z_range);
  const len_x = this.translateToScale(this.square_mode ? this.x_range/this.longest_axis * this.longest_axis : this.x_range);
  const len_y = this.translateToScale(this.square_mode ? this.y_range/this.longest_axis * this.longest_axis : this.y_range);

  this.points.forEach((otherPoint) => {
    let value = otherPoint.val || 0;
    //rescale
    let x = ((otherPoint.x - this.point_limits.min.x) * (len_x)) / (this.point_limits.max.x - this.point_limits.min.x)
    let y = ((otherPoint.y - this.point_limits.min.y) * (len_y)) / (this.point_limits.max.y - this.point_limits.min.y)
    let z = ((otherPoint.z - this.point_limits.min.z) * (len_z)) / (this.point_limits.max.z - this.point_limits.min.z)

    //console.log(point, {x,y,z})
    let distance = calculateDistance(point, {x,y,z});
    //let weight = 1/(distance*distance);
    //let weight = Math.pow(100,-(distance*distance))
    //let weight = 1/(distance*distance*distance);
    //let weight = 1/distance
    if(distance == 0){
      return value;
    }
    let weight = Math.exp(-distance*distance);
    totalWeight += weight;
      totalweightedVal += value*weight;
    
  })

  return totalweightedVal/totalWeight
}



// rerenders plane if moved 
updatePlane() {

  if (!this.interpolation_plane) {
    return;
  }
  
  if (this.planeGroup) {
    this.scene.remove(this.planeGroup);
  }
  
  this.planeGroup = new THREE.Group();
  
  const len_x = this.translateToScale(this.square_mode ? this.longest_axis : this.x_range);
  const len_y = this.translateToScale(this.square_mode ? this.longest_axis : this.y_range);
  const len_z = this.translateToScale(this.square_mode ? this.longest_axis : this.z_range);
  let z = this.displacement * len_z / 10
  let remove_points = true;

  if(this.remove_points){
    this.scene.remove(this.pointCloud)
    const new_points  = this.points.filter((e)=>{
      const len_z = this.translateToScale(this.square_mode ? this.z_range/this.longest_axis * this.longest_axis : this.z_range);
      let _z = ((e.z - this.point_limits.min.z) * (len_z)) / (this.point_limits.max.z - this.point_limits.min.z)
      return _z+this.sphere_radius < z

    })

    this.createPointCloud(new_points);
    this.shiftDown(this.pointCloud);
    this.initPos(this.pointCloud);


  }

  const countXvertices = 100;
  const countYvertices = 100;

  const canvas = document.createElement('canvas');
  canvas.width = countXvertices;
  canvas.height = countYvertices;
  const context = canvas.getContext('2d');

  for (let Y = 0; Y <= countYvertices; Y++) {
    for (let X = 0; X <= countXvertices; X++) {
      let x = ((X) * (len_x)) / countXvertices
      let y = ((countXvertices-Y) * (len_y)) / countXvertices
      let val = this.distanceToColor({ x, y, z });
      let color = this.getColor(this.range.maxVal, this.range.minVal, val);
      context.fillStyle = color;
      //context.fillStyle = `rgb(${x*10} ${0} ${0})`;
      context.fillRect(X, Y, 1, 1);
    }
  }


  const texture = new THREE.CanvasTexture(canvas);

  const planeGeometry = new THREE.PlaneGeometry(len_x, len_y, countXvertices, countYvertices);
  planeGeometry.translate(+len_x / 2, +len_y / 2, 0);
  const planeMaterial = new THREE.MeshBasicMaterial({ side: THREE.DoubleSide, map: texture });

  const planeMesh = new THREE.Mesh(planeGeometry, planeMaterial);
  var container = new THREE.Object3D();
  container.add(planeMesh);
  this.planeGroup.add(container);
  this.shiftDown(this.planeGroup);
  this.initPos(this.planeGroup);
  planeMesh.position.z += z;

  this.planeMesh = planeMesh;

  this.scene.add(this.planeGroup);
}


  //function to draw generic color scales in discrete space option x is not really finished because it does not consider the width of the text so it might overlap
  drawColorScale(range, direction, height, width, calculateGridLines, fontSize = '12px Arial', space=4) {
    const parent = this.wrapper
    const canvas = document.createElement('canvas');
    canvas.width = width;
    canvas.height = height;
    parent.appendChild(canvas);
    const ctx = canvas.getContext('2d');
    ctx.font = fontSize;
    let maxTextWidth = Math.max(ctx.measureText(range[0]).width, ctx.measureText(range[1]).width);
    let maxTextHeight = parseInt(fontSize, 10);
  
    if (direction === 'y') {
        maxTextWidth += space
        for (let i = maxTextHeight/2; i < height-maxTextHeight/2; i++) {
            const value = range[1] - Math.round(i / (height - maxTextHeight/2) * (range[1] - range[0]));
            const color = this.getColor(range[1],range[0], value);
            ctx.fillStyle = color;
            ctx.fillRect(maxTextWidth, i, width - maxTextWidth, 1);
        }
  
        
        const subdivisions = Math.floor(height / 20); 
        const step = calculateGridLines(range[1] - range[0], subdivisions);
  
        for (let i = range[0]; i <= range[1]; i += step) {
            const y = Math.round(height-maxTextHeight/2 - (i - range[0]) * (height-maxTextHeight) / (range[1] - range[0]));
            ctx.strokeStyle = '#000';
            ctx.beginPath();
            ctx.moveTo(maxTextWidth-space/2, y);
            ctx.lineTo(width, y);
            ctx.stroke();
            ctx.fillStyle = '#000';
            ctx.textAlign = 'end';
            ctx.textBaseline = 'middle';
            ctx.fillText(i, maxTextWidth-space, y);
        }
      } else if (direction === 'x') {
        maxTextHeight += space
        for (let i = maxTextWidth/2; i < width - maxTextWidth/2; i++) {
            const value = Math.round( i / (width - maxTextWidth/2) * (range[1] - range[0]) + range[0] );
            const color = this.getColor(range[1], range[0], value);
            ctx.fillStyle = color;
            ctx.fillRect(i, 0, 1, height - maxTextHeight);
        }
  
        const subdivisions = Math.floor(width / 20);
        const step = calculateGridLines(range[1] - range[0], subdivisions);
  
        for (let i = range[0]; i <= range[1]; i += step) {
            const x = Math.round((i - range[0]) / (range[1] - range[0]) * (width-maxTextWidth) + maxTextWidth/2);
            ctx.strokeStyle = '#000'; 
            ctx.beginPath();
            ctx.moveTo(x, 0);
            ctx.lineTo(x, height-maxTextHeight + space/2);
            ctx.stroke();
  
            ctx.fillStyle = '#000';
            ctx.textAlign = 'center';
            ctx.textBaseline = 'middle';
            const textY = height - maxTextHeight / 2;
            const textX = x;
            ctx.fillText(i, textX, textY+space);
        }
    }
  
    return canvas
  }
  
  

}



var width = screen.width > screen.height ? screen.width*0.6 : screen.width*0.9
var height = screen.height > screen.width ? screen.height * 0.45 : screen.height*0.9





// Verwendung: colorLambda(range, value);

var cloud_query = QS_PARAMS ? 'data?type='+QS_PARAMS : 'data';
getData(cloud_query).then(res => {new PointCloud3D("wrapper", width, height, JSON.parse(res)['data'], activeColorLambda)});

//new PointCloud3D("wrapper",width, height, examplePoints, (range, value) => [value * 255 / range, value * 0.6 * 255 / range, 0.25 * value * 255 / range]);

