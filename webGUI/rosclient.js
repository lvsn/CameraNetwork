/**
 * @author Mathieu Garon
 */


//   ----  ROS Initialisation  ----
var ros = new ROSLIB.Ros({
    url : 'ws://pimaster.jflalonde.org:9090'
});

//   ---- Class ---- 
 
function network_timelaps(){
	this.status = new ROSLIB.Topic({
		ros : ros,
		name : '/master/network_timelaps/status',
		messageType : 'actionlib_msgs/GoalStatusArray',
	});
	this.feedback = new ROSLIB.Topic({
		ros : ros,
		name : '/master/network_timelaps/feedback',
		messageType : 'camera_network_msgs/CameraControlActionFeedback'
	});
	this.result = new ROSLIB.Topic({
		ros : ros,
		name : '/master/network_timelaps/result',
		messageType : 'camera_network_msgs/CameraControlActionResult',
	});	
	this.action = new ROSLIB.ActionClient({
      	ros : ros,
   	  	serverName : '/master/network_timelaps',
      	actionName : 'camera_network_msgs/CameraControlAction'
    }); 
    
    this.setAction = function(form){
		var goal = new ROSLIB.Goal({
			actionClient : _network_timelaps.action,
			goalMessage : {
				picture_qty : parseFloat(form.network_timelaps_qty.value),
				inter_picture_delay_s : parseFloat(form.network_timelaps_frequency.value),
				is_hdr : $('#network_hdr').is(':checked')
			}
		});
		goal.send();
	}  	
	
	this.stopAction = function(){
		this.action.cancel();
	}
	
	this.feedback.subscribe(function(msg){
		$("#network_timelaps_feedback").text(msg.feedback.picture_taken);
	});
	this.status.subscribe(function(message) {
		statusList = message.status_list;
		if(statusList.length > 0){
			//console.log(message.status_list[0].status);
			$("#network_timelaps_status").text("Busy");
			$("#network_timelaps_status").css("color","red");
		}
		else{
			$("#network_timelaps_status").text("Idle");
			$("#network_timelaps_status").css("color","green");
		}
	}); 
	this.result.subscribe(function(msg){
		$("#network_timelaps_result").text(msg.result.total_picture);
	});
};      

function network_download(){
	this.status = new ROSLIB.Topic({
		ros : ros,
		name : '/master/sftp/status',
		messageType : 'actionlib_msgs/GoalStatusArray'
	});
	this.feedback = new ROSLIB.Topic({
		ros : ros,
		name : '/master/sftp/feedback',
		messageType : 'camera_network_msgs/CameraDownloadActionFeedback'
	});
	this.result = new ROSLIB.Topic({
		ros : ros,
		name : '/master/sftp/result',
		messageType : 'camera_network_msgs/CameraDownloadActionResult'
	});
	this.action = new ROSLIB.ActionClient({
      	ros : ros,
   	  	serverName : '/master/sftp',
      	actionName : 'camera_network_msgs/CameraDownloadAction'
    }); 
	
	this.setAction = function sendGoal(form){
		var goal = new ROSLIB.Goal({
			actionClient : _network_download.action,
			goalMessage : {
				dowload_frequency_s : parseFloat(form.network_download_frequency.value)
			}
		});
		goal.send();
	}  

    this.stopAction = function(){
		this.action.cancel();
	}
   
	this.feedback.subscribe(function(msg){
		$("#network_download_feedback").text(msg.feedback.picture_downloaded);
	});

	this.result.subscribe(function(msg){
		$("#network_download_result").text(msg.result.total_downloaded);
	});
	
	this.status.subscribe(function(message) {
		statusList = message.status_list;
		if(statusList.length > 0){
			//console.log(message.status_list[0].status);
			$("#network_download_status").text("Busy");
			$("#network_download_status").css("color","red");
		}
		else{
			$("#network_download_status").text("Idle");
			$("#network_download_status").css("color","green");
		}
		
	}); 
};

function device(){

	var param;
	var status;
	var feedback;
	var result;
	var action;
	var name;
	var ip;

	this.createRosAttribute = function(pName,pIP){
		name = pName;
		ip = pIP;
		param = new ROSLIB.Param({
	    	ros : ros,
			name : '/' + name
		}); 
		status = new ROSLIB.Topic({
			ros : ros,
			name :  name + '/timelaps/status',
			messageType : 'actionlib_msgs/GoalStatusArray'
		});
		feedback = new ROSLIB.Topic({
			ros : ros,
			name : name + '/timelaps/feedback',
			messageType : 'camera_network_msgs/CameraControlActionFeedback'
		});
		result = new ROSLIB.Topic({
			ros : ros,
			name : name + '/timelaps/result',
			messageType : 'camera_network_msgs/CameraControlActionResult'
		});
		action = new ROSLIB.ActionClient({
			ros : ros,
			serverName : name + '/timelaps',
			actionName : 'camera_network_msgs/CameraControlAction'  
	  	});  
		
		feedback.subscribe(function(msg){
			$("#device_timelaps_feedback").text(msg.feedback.picture_taken);
		});

		result.subscribe(function(msg){
			$("#device_timelaps_result").text(msg.result.total_picture);
		});	
		
	    status.subscribe(function(message) {
			statusList = message.status_list;
			if(statusList.length > 0){
				//console.log(message.status_list[0].status);
				$("#device_timelaps_status").text("Busy");
				$("#device_timelaps_status").css("color","red");
			}
			else{
				$("#device_timelaps_status").text("Idle");
				$("#device_timelaps_status").css("color","green");
			}
		});  
	
	}

	this.setAction = function sendGoal(form){
		var goal = new ROSLIB.Goal({
			actionClient : action,
			goalMessage : {
				picture_qty : parseFloat(form.device_timelaps_qty.value),
				inter_picture_delay_s : parseFloat(form.device_timelaps_frequency.value),
				is_hdr : $('#device_hdr').is(':checked')
			}
		});
		goal.send();
	}  

	this.stopAction = function(){
		action.cancel();
	}

    this.removeSequence = function() {
	    var setting = new ROSLIB.Param({
		    ros : ros,
		    name : name + '/camera_setting/captureSequence'
	    });
		setting.set([{}]);
	} 

	this.saveSequence = function(){
		var save = new ROSLIB.Service({
			ros : ros,
			name : '/' + name +'/save_config',
			serviceType : 'std_srvs/Empty'
		});
		var request = new ROSLIB.ServiceRequest({});
		save.callService(request, function(result) {});
	}
	
	this.streamVideo = function(){
		var stream_srv = new ROSLIB.Service({
			ros : ros,
			name : '/' + name + '/stream_video',
			serviceType : 'camera_network_msgs/VideoStream'
		});
		var request = new ROSLIB.ServiceRequest({
			frames : parseInt($("#device_stream_frames").val())
		});
		stream_srv.callService(request,function(result){});
	}

	this.addSequence = function(form){
		var setting = new ROSLIB.Param({
			ros : ros,
			name : name + '/camera_setting/captureSequence'
		});
		var config = {};
		if(form.device_sequence_shutterspeed.value != ''){
			config['shutterspeed']= form.device_sequence_shutterspeed.value
		}  
		if(form.device_sequence_aperture.value != ''){
			config['aperture']= form.device_sequence_aperture.value
		}     
		setting.get(function(value){
			if(JSON.stringify(value) === JSON.stringify([{}]) ){
				setting.set([config]); 
			}
			else{
				value.push(config)
				setting.set(value);    
			}
		});
		
	}

	this.setParameters = function(form){
		var settingIso = new ROSLIB.Param({
			ros : ros,
			name : name + '/camera_setting/iso'
		});
		var settingAperture = new ROSLIB.Param({
			ros : ros,
			name : name + '/camera_setting/aperture'
		});
		var settingShutterspeed = new ROSLIB.Param({
			ros : ros,
			name : name + '/camera_setting/shutterspeed'
		});
		var settingImageformat = new ROSLIB.Param({
			ros : ros,
			name : name + '/camera_setting/imageformat'
		});
		
		iso = form.device_parameter_iso.value;
		aperture = form.device_parameter_aperture.value;
		shutterspeed = form.device_parameter_shutterspeed.value;
		imageformat = form.device_parameter_imageformat.value;
		config = {};
		if(iso != ""){
			settingIso.set(iso);
		}
		if(aperture != ""){
			settingAperture.set(aperture);
		} 
		if(shutterspeed != ""){
			settingShutterspeed.set(shutterspeed);
		}
		if(imageformat != ""){
			settingImageformat.set(imageformat);
		}

	}

	this.getInformation = function(form){
		var save = new ROSLIB.Service({
			ros : ros,
			name : '/' + name +'/get_camera',
			serviceType : 'camera_network_msgs/OutCameraData'
		});
		var request = new ROSLIB.ServiceRequest({getAllInformation:true});
		save.callService(request, function(result) {
			string = '';
			string += result['iso'] + '\n';
			string += result['aperture'] + '\n';
			string += result['shutterspeed'] + '\n';
			string += result['imageformat'] + '\n';
			alert(string);
		});
	}

	this.refresh = function(){
		$("#device_name").text(name);
    	$("#device_ip").text(ip);
		param.get(function(value){
	    	if(value != null && value["camera_model"] != null){
	    		$("#device_camera").text(value["camera_model"]);
	    		$("#device_camera").css("color","black");
	    		$("#device_iso").text(value["camera_setting"]["iso"]);
	    		$("#device_aperture").text(value["camera_setting"]["aperture"]);
	    		$("#device_shutterspeed").text(value["camera_setting"]["shutterspeed"]);
	    		$("#device_imageformat").text(value["camera_setting"]["imageformat"]);
	    		
	    		var sequenceSize = value["camera_setting"]["captureSequence"].length;
	            var sequence = value["camera_setting"]["captureSequence"];
				var parameterString = '';
				for(var i = 0; i < sequenceSize; i++){
					parameterString += "Picture " + i + " :</br>";
				    for (var prop in sequence[i]){
				    	parameterString += prop + ": " + sequence[i][prop] + "</br>";
				   	}
				   	parameterString += "</br>";
				 }
				 $("#device_parameters").html(parameterString);
				 
	    	}
	    	else{
	          	$("#device_camera").text("No Camera");
	          	$("#device_camera").css("color","red");
	        }
	    });

	}
	
	this.clean = function(){
		status.unsubscribe();
	}
   
    this.drawPreview = function(form){
		var preview = new ROSLIB.Service({
			ros : ros,
			name : '/' + name + '/preview_camera',
			serviceType : 'std_srvs/Empty'
		});
		var request = new ROSLIB.ServiceRequest({});
	  preview.callService(request);
	  
	}

	this.getIp = function(){
		return ip;
	}
}

//   ----  Instantces   -----

var _network_timelaps = new network_timelaps();
var _network_download = new network_download();
var _current_device;
var img = new Image;
img.src = "http://pimaster.jflalonde.org:8181/stream?topic=/preview?width=640?height=480";

//   ---   Functions   ----

function cleanDevicePage(){
	$("#device_name").text("");
	$("#device_ip").text("");
	$("#device_camera").text("");
	$("#device_iso").text("");
	$("#device_aperture").text("");
	$("#device_shutterspeed").text("");
	$("#device_imageformat").text("");
	$("#device_parameters").text("");
	$("#device_timelaps_status").text("");
	$("#device_timelaps_result").text("");
	$("#device_timelaps_feedback").text("");
}

function refreshScreen(){
	if (_current_device == undefined){
		cleanDevicePage();
	}
	else{
		_current_device.refresh();
	}
}

function refreshCanvas(){
    document.getElementById('imagePreview').src = "http://pimaster.jflalonde.org:8181/stream?topic=/preview?width=640?height=480";
};

function refreshSelect(){
	IpList = new ROSLIB.Param({
		ros : ros,
		name : '/IP'
	});
	IpList.get(function(value) {
		$("#deviceList").empty(); 
		var select = document.getElementById("deviceList");
		select.options[0] = new Option("Online Devices", "index0");
		$.each( value, function( key, value ) {
			select.options[select.options.length] = new Option(key, value);
		});
		if(_current_device != undefined){
			$("#deviceList").val(_current_device.getIp());	
		}
	});
}

function noDeviceAlert(){
	if(_current_device == undefined){
		alert("No device selected");
		return true;
	}
	else{
		return false;
	}
}

// ---- events ----

function selectEvent(select){
	if(select.selectedIndex != 0){
		currentDevice = select.options[select.selectedIndex].text;
		_current_device = new device();
		_current_device.createRosAttribute(currentDevice,select.value);
		_current_device.refresh();
	}
	else{
		_current_device.clean();
		_current_device = undefined;
	}
}

function networkTimelapsEvent(form,isStart){
	if(isStart){
		_network_timelaps.setAction(form);
	}
	else{
		_network_timelaps.stopAction();
	}
}

function networkDownloadEvent(form,isStart){
	if(isStart){
		_network_download.setAction(form);
	}
	else{
		_network_download.stopAction();
	}
}

function setParametersEvent(form){
	if(!noDeviceAlert()){
		_current_device.setParameters(form);		
	}
}

function getInformationEvent(form){
	if(!noDeviceAlert()){
		_current_device.getInformation();		
	}
}

function removeSequenceEvent(form){
	if(!noDeviceAlert()){
		_current_device.removeSequence();		
	}
}

function addSequenceEvent(form){
	if(!noDeviceAlert()){
		_current_device.addSequence(form);		
	}
}

function saveSequenceEvent(form){
	if(!noDeviceAlert()){
		_current_device.saveSequence();		
	}
}

function deviceTimelapsEvent(form,isStart){
	if(isStart){
		_current_device.setAction(form);
	}
	else{
		_current_device.stopAction();
	}
}

function streamVideoEvent(form){
	if(!noDeviceAlert()){
		_current_device.streamVideo();		
	}
}

function drawPreviewEvent(form){
	if(!noDeviceAlert()){
		_current_device.drawPreview();		
	}
}

//    ----   Dynamic page   -----

setInterval(refreshScreen, 1000);
setInterval(refreshCanvas, 500);
setInterval(refreshSelect,2000);
