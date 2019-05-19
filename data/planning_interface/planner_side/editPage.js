var nodeInfoForm = '<div id="actualName"></div>'+
            '<div class="form-horizontal" id="dataForm"><div class="form-group">'+
						'<label class="col-md-4 control-label" for="controllabilityTag">Controllability Tag</label>'+
						'<div class="col-md-4"><select id="controllabilityTag" name="controllabilityTag" class="form-control"><option value="c">Controllable</option><option value="u">Uncontrollable</option></select></div></div>'+

					'<div class="form-group"><label class="col-md-4 control-label" for="durability">Durability</label><div class="col-md-4"><input id="durability" name="durability" type="text" placeholder="Es: 44-80" class="form-control input-md" style="width:150px;"></div></div>'+

					'<div class="form-group"><label class="col-md-4 control-label" for="isInitial">Is initial state</label><div class="col-md-4"><input id="isInitial" name="isInitial" type="checkbox" class="form-check-input"></div></div>'+

					'<button class="btn btn-primary" onclick="getFormData()" style="width:250px;">Add info</button></div>';

var newStateVar = JSON.parse(window.localStorage.getItem('newStateVar'));

var allStatesInfo = JSON.parse(window.localStorage.getItem('allStatesInfo'));//the scope is @global to let modify the states

var conversionDict = JSON.parse(window.localStorage.getItem('conversionDict')); //Dictionary to handle sync rule rewriting in accord to merging

function drawModifiedGraph(){
  /*
  * Draws the graph using a Sticky Force Layout. Why sticky? To better handle and understand the graph
  */

  //Display all synchronization constraints
  if (newStateVar.hasOwnProperty('syncRule'))
    displaySyncRule(newStateVar.syncRule);
  else console.log('No synchronization rules found.');


  //console.log(newStateVar);
  var links = newStateVar.links;

  var workArea = d3.select('#modifyArea');
  var nodes = {};
  var w = 800,
  h = 400;

  var svg = workArea.select("svg")
    .attr("width", w)
   .attr("height", h);

  links.forEach(function(link) {
    link.source = nodes[link.source] || (nodes[link.source] = {name: link.source});
    link.target = nodes[link.target] || (nodes[link.target] = {name: link.target});
  });

  var force = d3.layout.force()
  .nodes(d3.values(nodes))
  .links(links)
  .size([w, h])
  .gravity(0.5)
  .linkDistance(200)
  .charge(-800)
  .on("tick", tick)
  .start();

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
  var link = svg.append("svg:g").selectAll(".path")
    .data(force.links())
  .enter().append("svg:path")
    .attr("class", "link")
    .attr("marker-end", "url(#end)");

  //sticky force layout
  var drag = force.drag()
              .on('dragstart', function(d){
                d3.select(this).classed("fixed", d.fixed = true);
              })


   var node = svg.selectAll('.node')
  .data(d3.values(nodes))
  .enter().append('g')
  .attr('class','node')
  .call(drag);

  node.append('circle')
  .style("fill", "red")
  .attr("r", 9);

  node.append('text')
  .attr("x", 25)
  .attr('dy','.80em')
  .text(function(d){ return d.name; });

  var tooltip = d3.select('#modifyArea')
      .append('div')
      .attr('class','tooltip')
      .style('opacity',0);

  node.on('click',function(d){
    d3.select('#nodeEditing')
      .html(nodeInfoForm);
    d3.select('#actualName')
    .select('text')
    .remove();
    d3.select('#actualName')
    .append('text')
    //.attr("x", 25)
    //.attr('dy','.80em')
    .text(d.name);
  })
  .on('mouseover', function(d){
    var infos;
    allStatesInfo.forEach(function(s){
      if (d.name == s.id){
        infos = JSON.stringify(s);
      }
    })
    tooltip.html('<strong>Node Info</strong><br>'+infos)
      .style('top', (d3.event.pageY) + 'px')
      .style('left', (d3.event.pageX - 50) + 'px')
      .style("opacity", 1);
  })
  .on('mouseout', function(d){
    tooltip.style('opacity',0);
  })

  var tooltip = d3.select('#modifyArea')
      .append('div')
      .attr('class','tooltip')
      .style('opacity',0);

  node.on('mouseover', function(d){
    var infos;
    allStatesInfo.forEach(function(s){
      if (d.name == s.id){
        infos = JSON.stringify(s);
      }
    })

    tooltip.html('<strong>Node Info</strong><br>'+infos)
      .style("opacity", 1);
  })
  .on('mouseout', function(d){
    tooltip.style('opacity',0);
  })

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

  d3.select('#savePanel')
    .append('input')
      .attr('type','button')
      .attr('class','button')
      .style('background-color','#82b74b')
      .attr('value', 'Add SV to WS')
      .on('click', function(d){
        /**Creating new state var to add**/
        var editedStateVar = {};
        editedStateVar.stateVar = newStateVar.stateVar;
        editedStateVar.flag = 'edited';
        var states = [];
        d3.values(nodes).forEach(function(d){
          var addingState = allStatesInfo.filter(function(s){return s.id == d.name});
          if (addingState.length > 0){
            states.push(addingState[0]);
          }
        });
        editedStateVar.states=states;
        var newLinks = [];
        links.forEach(function(d){
          newLinks.push({source: d.source.name, target: d.target.name, topic: d.topic, message: d.message, type:d.type});
        });
        editedStateVar.links=newLinks;
        var editedSV = JSON.parse(window.localStorage.getItem('stateVariables'));
        editedSV.push(editedStateVar);
        window.localStorage.setItem('stateVariables',JSON.stringify(editedSV));
        window.close();
      });
}

function getFormData(){
  /*
  * Get input for selected node and update info
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

  var found = false;
  allStatesInfo.forEach(function(d){
    if (d.id == name){
      console.log('this is' + d.id +' , '+ name)
      d.controlTag = tag;
      d.durability = dur;
      d.is_initial = init;
      found = true;
    }
  });
  if (!found)
      allStatesInfo.push({id: name, controlTag: tag, durability: dur, is_initial: init}); 
  window.localStorage.setItem('allStatesInfo',JSON.stringify(allStatesInfo));
}

function displaySyncRule(rules){
  //var newRules = convertRules(rules);
  d3.select('body').select('#syncRule')
  .selectAll('p')
  .data(rules).enter()
  .append('p')
  .append('text')
  .text(function(d){return d}
    );
  }

/******************************************************************************/
function convertRules(oldRules){
  dict = buildSimpleDict(conversionDict);
  var newRules = [];
  for (index in oldRules){
    var temp = oldRules[index];
    var elements = temp.split(' ');
    /*How is elements composed: ["ION", "->", "I", "BEFORE", "Akka", "->","_K"]
    *elements[0] = SV Stato 1
    *elements[2] = Stato 1
    *elements[3] = Synchronization rule
    *elements[4] = SV Stato 2
    *elements[6] = Stato 2
    */
    //Assigning values to temporary variables
    var sv1 = elements[0], st1 = elements[2],
        sv2 = elements[4], st2 = elements[6];
    var rule = ''; //temporary rule that will beb built in the for loop
    if (dict[sv1] != undefined)
      rule += dict[sv1]+' -> '+dict[st1];
    else rule += sv1+' -> '+st1;
    rule += ' '+elements[3]+' ';
    if (dict[sv2]!= undefined)
      rule += dict[sv2]+' -> '+dict[st2];
    else rule += sv2+' -> '+st2;
    newRules.push(rule);
  }
  
  console.log(newRules);
  return newRules;
}

function buildSimpleDict(dict){
  simpleDict={};
  for (index in dict){
    var temp = dict[index];
    var key = temp.key;
    var val = temp.value;
    simpleDict[key] = val;
  }
  return simpleDict;
}

function printAll(list){
  for (index in list)
    console.log(list[index]);
}