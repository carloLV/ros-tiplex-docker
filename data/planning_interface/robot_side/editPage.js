var dataset = [];
var nodeNumber; // Need to check if all states have been edited

var nodeInfoForm = '<div class="form-horizontal" id="dataForm"><div class="form-group">'+
						'<label class="col-md-4 control-label" for="controllabilityTag">Controllability Tag</label>'+
						'<div class="col-md-4"><select id="controllabilityTag" name="controllabilityTag" class="form-control"><option value="c">Controllable</option><option value="u">Uncontrollable</option></select></div></div>'+

					'<div class="form-group"><label class="col-md-4 control-label" for="durability">Durability</label><div class="col-md-4"><input id="durability" name="durability" type="text" placeholder="Es: 44-80" class="form-control input-md" style="width:150px;"></div></div>'+

					'<div class="form-group"><label class="col-md-4 control-label" for="isInitial">Is initial state</label><div class="col-md-4"><input id="isInitial" name="isInitial" type="checkbox" class="form-check-input"></div></div>'+

					'<button class="btn btn-primary" onclick="getFormData()" style="width:250px;">Add info</button></div>';

function handleClick(event){
  /*
  * Handles input when adding states. Alerts if there are problems
  */
	var src = d3.select("#nodeVal").property('value');
	var tgt = d3.select(" #linksVal").property('value');
 		
 	if (src=='' || tgt==''){
		alert('Source and/or target cannot be empty');
		return
 		}
  draw(src, tgt, d3.select("#topics").property('value'), d3.select('input[name="group-radio"]:checked').property("value"));
}

function draw(src, tgt, tpc, role){
  /*
  * Draws the list containing the edges. 
  * @input source and target string for each edge of the graph
  */
  console.log(role);
  d3.select("body").select("ul").append("li");
  var top_msg_list = JSON.parse(window.localStorage.getItem('feedback'));
  var top_msg_el = top_msg_list.filter(function(element){ return element.topic == tpc; })
  dataset.push({source: src, target: tgt, topic: tpc, message: top_msg_el[0].message, type: role});
  var p = d3.select("body").selectAll("li")
  .data(dataset)
  .text(function(d,i){ return i + ": " + d.source+'-'+d.target;})
}

function getDataset(){
  /*
  * When edges are added, it creates the State Variable (graph) associated with this description
  */
	if (dataset.length<1){
		alert('You need at least 1 link')
		return
	}
	jsonInit();
  console.log(dataset)
	createGraph(dataset);

}

function jsonInit(){
  /*
  * Creates in the @localStorage the description of CURRENT state variable that is edited.
  */ 
  var obj = JSON.parse(window.localStorage.getItem(document.title));
  for (index in dataset){
  	var temp = dataset[index]
  	obj.links.push({source: temp.source, target: temp.target, topic: temp.topic, message: temp.message, type: temp.type})
  }
  obj.flag='edited';
  window.localStorage.setItem(document.title, JSON.stringify(obj))
  }

function createGraph(links){
  /*
  * Draws graph using passed links using Force Layout standards
  */

	if (d3.select("#chart").select("svg"))
		d3.select("#chart").select("svg").remove()

  var nodes = {};
  var w = 800,
  h = 350;

  console.log(links);
  
  links.forEach(function(link) {
    link.source = nodes[link.source] || (nodes[link.source] = {name: link.source});
    link.target = nodes[link.target] || (nodes[link.target] = {name: link.target});
  });
  nodeNumber = d3.values(nodes).length;

  var force = d3.layout.force()
  .nodes(d3.values(nodes))
  .links(links)
  .size([w, h])
  .gravity(0.5)
  .linkDistance(200)
  .charge(-800)
  .on("tick", tick)
  .start();

  var svg = d3.select("#chart").append("svg")
  .attr("width", w)
  .attr("height", h);

  // build the arrow.
svg.append("svg:defs").selectAll("marker")
    .data(["end"])      // Different link/path types can be defined here
  .enter().append("svg:marker")    // This section adds in the arrows
    .attr("id", String)
    .attr("viewBox", "0 -5 10 10")
    .attr("refX", 15)
    .attr("refY", -1.5)
    .attr("markerWidth", 6)
    .attr("markerHeight", 6)
    .attr("orient", "auto")
  .append("svg:path")
    .attr("d", "M0,-5L10,0L0,5");

// add the links and the arrows
var link = svg.append("svg:g").selectAll("path")
    .data(force.links())
  .enter().append("svg:path")
    .attr("class", "link")
    .attr("marker-end", "url(#end)");

   var node = svg.selectAll('.node')
  .data(d3.values(nodes))
  .enter().append('g')
  .attr('class','node')
  .call(force.drag);

  node.append('circle')
  .style("fill", "red")
  .attr("r", 18);

  node.append('text')
  .attr("x", 25)
  .attr('dy','.80em')
  .text(function(d){ return d.name; });

  //To display nodes name always:  https://stackoverflow.com/questions/11102795/d3-node-labeling

  node.on('click',function(d){
  	d3.select('#nodeEditing')
  		.html(nodeInfoForm);
  	d3.select('#actualName')
    .select('text')
    .remove();
  	d3.select('#actualName')
    .append('text')
    .text(d.name);
  })

  link.on('mouseover', function(d) {
  	d3.select('#topicEdge').property('value',d.topic)
  });
  
  function tick() {

    link
    .attr("d", function(d) {
        var dx = d.target.x - d.source.x,
            dy = d.target.y - d.source.y,
            dr = Math.sqrt(dx * dx + dy * dy);
        return "M" + 
            d.source.x + "," + 
            d.source.y + "A" + 
            dr + "," + dr + " 0 0,1 " + 
            d.target.x + "," + 
            d.target.y;
    });
    node
        .attr("transform", function(d) { 
  	    return "translate(" + d.x + "," + d.y + ")"; });
  }
}

function getFormData(){
  /*
  * When clicking on node, it shows a form to edit information about node. Clicking button adds info to node
  */

		var name = d3.select('#actualName')
	  .property('textContent');

	  var tag = d3.select('#dataForm')
	  .select('#controllabilityTag')
	  .property('value');
	  
	  var dur= d3.select('#dataForm')
	  .select('#durability')
	  .property('value');

	  var init = d3.select('#dataForm')
	  .select('#isInitial')
	  .property('checked');

  //Set up of states for the current State Variable, according to info from the form 
  var obj = JSON.parse(window.localStorage.getItem(document.title));

  //Push node if not present; if present then substitute values
  var found = false //If the state is already present
  for (index in obj.states){
    var temp = obj.states[index];
    if (temp.id == name){
      found = true
      temp. controlTag = tag;
      temp.durability = dur;
      temp.is_initial = init;
    }
  }
  if (!found)
    obj.states.push({id: name, controlTag: tag, durability: dur, is_initial: init});
  window.localStorage.setItem(document.title, JSON.stringify(obj));

  d3.selectAll('.node')
  	.filter(function(d){
  		return d.name==name})
  	.select('circle')
  	.style('fill', 'green');
}

function endEditing(){
	/*
  * If confirmed saves in memory the shown message
  */
	var data = window.localStorage.getItem(document.title);
	var sts = JSON.parse(data).states;
	console.log(sts.length, nodeNumber);
	if (sts.length != nodeNumber){
		alert('Please edit all nodes. Check console to know wich you edited');
		return
	}
	if (confirm('Your data:\n'+data))
		window.close();
}