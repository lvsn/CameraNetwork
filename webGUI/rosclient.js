/**
 * @author Mathieu Garon
 */

// Set the ROS_MASTER variable to the current server hostname
var ROS_MASTER = window.location.hostname;


//   ----  ROS Initialisation  ----
var ros = new ROSLIB.Ros({
    url : 'ws://' + ROS_MASTER + ':9090'
});

var ros_conn_state = 'NOT CONNECTED';

ros.on('connection', function() {
    $.unblockUI();
    ros_conn_state = 'CONNECTED';
});

ros.on('close', function() {
    ros_conn_state = 'NOT CONNECTED';
});

ros.on('error', function() {
    ros_conn_state = 'NOT CONNECTED';
});


//   ---- Class ---- 
 
function network_timelapse(){
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
    
    this.setAction = function(form) {
		var goal = new ROSLIB.Goal({
			actionClient : _network_timelapse.action,
			goalMessage : {
				picture_qty : parseFloat(form.network_timelapse_qty.value),
				inter_picture_delay_s : parseFloat(form.network_timelapse_frequency.value),
				is_hdr : $('#network_hdr').is(':checked')
			}
		});
		goal.send();
	}  	
	
	this.stopAction = function(){
		this.action.cancel();
	}
	
	this.feedback.subscribe(function(msg){
		$("#network_timelapse_feedback").text(msg.feedback.picture_taken);
	});
	this.status.subscribe(function(message) {
		statusList = message.status_list;
		if(statusList.length > 0){
			//console.log(message.status_list[0].status);
			$("#network_timelapse_status").text("Busy");
			$("#network_timelapse_status").css("color", "orange");
		}
		else{
			$("#network_timelapse_status").text("Idle");
			$("#network_timelapse_status").css("color", "green");
		}
	}); 
	this.result.subscribe(function(msg){
		$("#network_timelapse_result").text(msg.result.total_picture);
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
        var ts = $("#download_start_at").timepicker('getTime', [new Date()]);
        // If ts not defined or not a D:DD or DD:DD entry, where D is a digit
        if (ts == null || ts == undefined || $("#download_start_at").val().match(/\d+:\d\d/) == null) {
            ts = new Date();
            $("#download_start_at").timepicker('setTime', ts);
        }
        var goal = new ROSLIB.Goal({
            actionClient : _network_download.action,
            goalMessage : {
                dowload_frequency_s : parseFloat(form.network_download_frequency.value),
                start_time : ts.getTime()/1000
            }
        });
        goal.send();
    }  

    this.stopAction = function(){
		this.action.cancel();
	}

    this.addUser = function(form){
		var add = new ROSLIB.Service({
			ros : ros,
			name : '/master/add_user',
			serviceType : 'camera_network_msgs/User'
		});
		var request = new ROSLIB.ServiceRequest({
                                name : form.download_name.value,
                                username : form.download_username.value,
                                password : form.download_password.value

            });
		add.callService(request, function(result) {});
	}

    this.delUsers = function(form){
		var del = new ROSLIB.Service({
			ros : ros,
			name : '/master/delete_users',
			serviceType : 'std_srvs/Empty'
		});
		var request = new ROSLIB.ServiceRequest({});
		del.callService(request, function(result) {});
	}

    this.saveUsers = function(form){
		var save = new ROSLIB.Service({
			ros : ros,
			name : '/master/save_users',
			serviceType : 'std_srvs/Empty'
		});
		var request = new ROSLIB.ServiceRequest({});
		save.callService(request, function(result) {});
	}

    this.getUsers = function(form){
		var get = new ROSLIB.Service({
			ros : ros,
			name : '/master/get_users',
			serviceType : 'camera_network_msgs/BackMessage'
		});
		var request = new ROSLIB.ServiceRequest({});
		get.callService(request, function(result) {
                alert(result.message);
            });
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
			$("#network_download_status").css("color", "orange");
		}
		else{
			$("#network_download_status").text("Idle");
			$("#network_download_status").css("color", "green");
		}
		
	}); 
};

function setSelectOptions(selector, values, current_value, set_value_as_key) {
    for (var i = 0; i < values.length; i++) {
        var val;
        if (set_value_as_key != undefined && set_value_as_key == true) {
            val = values[i][0];
        } else {
            val = values[i][1];
        }
        $(selector)
            .append($("<option></option>")
                .attr("value", val)
                .text(values[i][1])
                .prop('selected', current_value == values[i][1] ? true : false));
    }
}

function getConfiguration(str) {
    /* Return value: First element is the current value, Second is the list */
    elements = str.split("\n");
    var choices = [];
    var current = 0;
    for (var i = 0; i < elements.length; i++) {
        if (elements[i].slice(0, 7) == "Current") {
            current = elements[i].split(":")[1].trim();
        } else if (elements[i].slice(0, 6) == "Choice") {
            data = elements[i].split(" ");
            choices[choices.length] = [data[1].trim(), data.slice(2, data.length).join().trim()];
        }
    }
    return [current, choices];
}

function device() {

	var param;
	var status;
	var feedback;
	var result;
	var action;
	var name;
	var ip;

	this.createRosAttribute = function(pName, pIP){
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
			$("#device_timelapse_feedback").text(msg.feedback.picture_taken);
		});

		result.subscribe(function(msg){
			$("#device_timelapse_result").text(msg.result.total_picture);
		});	
		
	    status.subscribe(function(message) {
			statusList = message.status_list;
			if(statusList.length > 0){
				//console.log(message.status_list[0].status);
				$("#device_timelapse_status").text("Busy");
				$("#device_timelapse_status").css("color", "orange");
			}
			else{
				$("#device_timelapse_status").text("Idle");
				$("#device_timelapse_status").css("color", "green");
			}
		});  
	
	}

	this.setAction = function sendGoal(form){
		var goal = new ROSLIB.Goal({
			actionClient : action,
			goalMessage : {
				picture_qty : parseFloat(form.device_timelapse_qty.value),
				inter_picture_delay_s : parseFloat(form.device_timelapse_frequency.value),
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
	
	this.shutdownDevice = function(){
		var shutdown = new ROSLIB.Service({
			ros : ros,
			name : '/' + name + '/shutdown_device',
			serviceType : 'camera_network_msgs/CommandOption'
		});
		var request = new ROSLIB.ServiceRequest({option:"-h"});
		shutdown.callService(request, function(result) {});
	}
	
	this.rebootDevice = function(){
		var shutdown = new ROSLIB.Service({
			ros : ros,
			name : '/' + name + '/shutdown_device',
			serviceType : 'camera_network_msgs/CommandOption'
		});
		var request = new ROSLIB.ServiceRequest({option:"-r"});
		shutdown.callService(request, function(result) {});
	}
	
	this.streamVideo = function(){
		var stream_srv = new ROSLIB.Service({
			ros : ros,
			name : '/' + name + '/stream_video',
			serviceType : 'camera_network_msgs/Uint32'
		});
		var request = new ROSLIB.ServiceRequest({
			integer : parseInt($("#device_stream_frames").val())
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
            var config = getConfiguration(result['iso']);
            setSelectOptions("#device_parameter_iso", config[1], config[0]);
            config = getConfiguration(result['aperture']);
            setSelectOptions("#device_parameter_aperture", config[1], config[0]);
            config = getConfiguration(result['shutterspeed']);
            setSelectOptions("#device_parameter_shutterspeed", config[1], config[0]);
            config = getConfiguration(result['imageformat']);
            setSelectOptions("#device_parameter_imageformat", config[1], config[0], true);
            $("#param_status").html('');
		});
	}

	this.refresh = function() {
		$("#device_name").text(name);
    	$("#device_ip").text(ip);
		param.get(function(value) {
	    	if (value != null && value["camera_model"] != null && value != undefined) {
	    		$("#device_camera").text(value["camera_model"]);
	    		$("#device_camera").css("color","black");
	    		$("#device_iso").text(value["camera_setting"]["iso"]);
	    		$("#device_aperture").text(value["camera_setting"]["aperture"]);
	    		$("#device_shutterspeed").text(value["camera_setting"]["shutterspeed"]);
	    		$("#device_imageformat").text(value["camera_setting"]["imageformat"]);
	    		
	    		var sequenceSize = value["camera_setting"]["captureSequence"].length;
	            var sequence = value["camera_setting"]["captureSequence"];
				var parameterString = '';
				for (var i = 0; i < sequenceSize; i++) {
					parameterString += "Picture " + i + " :</br>";
				    for (var prop in sequence[i]) {
				    	parameterString += prop + ": " + sequence[i][prop] + "</br>";
				   	}
				   	parameterString += "</br>";
				 }
				 $("#device_parameters").html(parameterString);
				 
	    	} else {
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

    this.calibrateVideo = function(form){
		var cal = new ROSLIB.Service({
			ros : ros,
			name : '/' + name + '/calibrate_video',
			serviceType : 'std_srvs/Empty'
		});
		var request = new ROSLIB.ServiceRequest({});
	  cal.callService(request);
	  
	}

    this.calibratePicture = function(form){
		var cal = new ROSLIB.Service({
			ros : ros,
			name : '/' + name + '/calibrate_device',
			serviceType : 'std_srvs/Empty'
		});
		var request = new ROSLIB.ServiceRequest({});
	  cal.callService(request);
    }

	this.getIp = function(){
		return ip;
	}
}

//   ----  Instantces   -----

var _network_timelapse = new network_timelapse();
var _network_download = new network_download();
var _current_device;
//var img = new Image;
//img.src = "http://" + ROS_MASTER + ":8181/stream?topic=/preview?width=640?height=480";

//   ---   GUI Functions   ----

function cleanDevicePage(){
	$("#device_name").text("");
	$("#device_ip").text("");
	$("#device_camera").text("");
	$("#device_iso").text("");
	$("#device_aperture").text("");
	$("#device_shutterspeed").text("");
	$("#device_imageformat").text("");
	$("#device_parameters").text("");
    $("#device_timelapse_status").text("Awaiting connection...");
    $("#device_timelapse_status").css("color", "red");
	$("#device_timelapse_result").text("");
	$("#device_timelapse_feedback").text("");
    /* Parameters */
    $("#device_parameter_iso option").remove()
    $("#device_parameter_aperture option").remove()
    $("#device_parameter_shutterspeed option").remove()
    $("#device_parameter_imageformat option").remove()
}

function refreshScreen(){
	if (_current_device == undefined){
		cleanDevicePage();
	}
     else{
          _current_device.refresh();
	}
}


function refreshSelect(){
	IpList = new ROSLIB.Param({
		ros : ros,
		name : '/IP'
	});
	IpList.get(function(result) {
		var select = $("#deviceList");
        var old_value = select.val();
        $("#deviceList option").remove();
        select.prop("disabled", false);

		$.each( result, function( key, value ) {
            select.append($("<option></option>")
                    .attr("value", value)
                    .text(key));
		});

        /* Set currently selected device to previously selected one */
		if (_current_device != undefined) {
			$("#deviceList").val(_current_device.getIp());	
		}

        /* Trigger onChange event */
        if (select.val() != old_value) {
            select.change();
        }

        /* Only start the timer after a request is received */
        setTimeout(refreshSelect, 2000);
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

function selectEvent(select) {
    if ($(select).val() == null || $(select).val() == undefined) {
        return;
    }
    currentDevice = select.options[select.selectedIndex].text;
    _current_device = new device();
    _current_device.createRosAttribute(currentDevice,select.value);
    _current_device.refresh();
    getDeviceInformation();
}

function networkTimelapsEvent(form,isStart){
	if(isStart){
		_network_timelapse.setAction(form);
	}
	else{
		_network_timelapse.stopAction();
	}
}

var cmdVideo = new ROSLIB.Topic({
    ros : ros,
    name : '/network_capture_video_chatter',
    messageType : 'std_msgs/UInt32'
});

function networkVideoEvent(form){

    var msg = new ROSLIB.Message({
        data : parseInt(form.video_time.value)
    });
    cmdVideo.publish(msg);

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

function deviceTimelapsEvent(form, isStart){
	if (isStart) {
		_current_device.setAction(form);
	} else{
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

function calibrateVideoEvent(form){
	if(!noDeviceAlert()){
		_current_device.calibrateVideo();		
	}
}

function calibratePictureEvent(form){
	if(!noDeviceAlert()){
		_current_device.calibratePicture();		
	}
}

function shutdownDeviceEvent(form){
	if(!noDeviceAlert()){
		var r = confirm("Are you sure you want to shudown device?");
		if (r == true) {
		    _current_device.shutdownDevice();
		    _current_device.clean();
			_current_device = undefined;
		}		
	}
}

function rebootDeviceEvent(form){
    if(!noDeviceAlert()){
        var r = confirm("Are you sure you want to reboot device?");
        if (r == true) {
            _current_device.rebootDevice();
            _current_device.clean();
            _current_device = undefined;
        }
    }
}

function getDeviceInformation() {
	if(!noDeviceAlert()){
        $("#param_status").html('<img src="./media/loading.gif" /> Loading configuration...');
		_current_device.getInformation();		
	}
}


function initPreview(){
    mjpegSrc = ["http://",
                ROS_MASTER,
                ":8181/stream?topic=/preview?width=640?height=480"
                ].join('');
    $("#preview_image").attr("src",mjpegSrc);
};

$(document).ready(function() {
    if (ros_conn_state != 'CONNECTED') {
        $.blockUI({ message: '<img src="./media/loading.gif" /> Connecting to ROS...' });
    }
    if ($("#deviceList").length != 0) {
        refreshSelect();
    }
    if($("#preview_image").length != 0){
        initPreview();
    }
    /* refreshCanvas overloads the server */
    //var refreshCanvasTimer = setInterval(refreshCanvas, 100);

    var refreshScreenTimer = setInterval(refreshScreen, 1000);

});
