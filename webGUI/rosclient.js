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
 
function network_timelapse()
{
	this.status = new ROSLIB.Topic({
		ros : ros,
		name : '/master/network_timelapse/status',
		messageType : 'actionlib_msgs/GoalStatusArray',
	});
	this.feedback = new ROSLIB.Topic({
		ros : ros,
		name : '/master/network_timelapse/feedback',
		messageType : 'camera_network_msgs/CameraControlActionFeedback'
	});
	this.result = new ROSLIB.Topic({
		ros : ros,
		name : '/master/network_timelapse/result',
		messageType : 'camera_network_msgs/CameraControlActionResult',
	});	
	this.action = new ROSLIB.ActionClient({
      	ros : ros,
   	  	serverName : '/master/network_timelapse',
      	actionName : 'camera_network_msgs/CameraControlAction'
    }); 
    
    this.setAction = function(form) {
		var goal = new ROSLIB.Goal({
			actionClient : _network_timelapse.action,
			goalMessage : {
				picture_qty : parseFloat(form.network_timelapse_qty.value),
				inter_picture_delay_s : parseFloat(form.network_timelapse_frequency.value),
				mode : parseInt($('input[name=capture_type_net]:checked', 'form[name=network]').val())
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

function setSelectOptions(selector, values, current_value) {
    for (var i = 0; i < values.length; i++) {
        var val;
        val = values[i][1];
        $(selector)
            .append($("<option></option>")
                .attr("value", val)
                .text(values[i][1])
                .prop('selected', current_value == values[i][1] ? true : false));
    }
}

function getConfiguration(str)
{
    /* Return value: First element is the current value, Second is the list */
    elements = str.split("\n");
    var choices = [];
    var current = 0;
    for (var i = 0; i < elements.length; i++) {
        if (elements[i].slice(0, 7) == "Current") {
            current = elements[i].split(":")[1].trim();
        } else if (elements[i].slice(0, 6) == "Choice") {
            data = elements[i].split(" ");
            choices[choices.length] = [data[1].trim(), data.slice(2, data.length).join(" ").trim()];
        }
    }
    return [current, choices];
}

function device()
{
	var param;
	var status;
	var feedback;
	var result;
	var action;
	var name;
	var ip;
	var output;

	this.createRosAttribute = function(pName, pIP){
		name = pName;
		ip = pIP;
		param = new ROSLIB.Param({
	    	ros : ros,
			name : '/' + name
		}); 
		status = new ROSLIB.Topic({
			ros : ros,
			name :  name + '/timelapse/status',
			messageType : 'actionlib_msgs/GoalStatusArray'
		});
		feedback = new ROSLIB.Topic({
			ros : ros,
			name : name + '/timelapse/feedback',
			messageType : 'camera_network_msgs/CameraControlActionFeedback'
		});
		result = new ROSLIB.Topic({
			ros : ros,
			name : name + '/timelapse/result',
			messageType : 'camera_network_msgs/CameraControlActionResult'
		});
		action = new ROSLIB.ActionClient({
			ros : ros,
			serverName : name + '/timelapse',
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
				$("#device_timelapse_status").text("Busy");
				$("#device_timelapse_status").css("color", "orange");
			}
			else{
				$("#device_timelapse_status").text("Idle");
				$("#device_timelapse_status").css("color", "green");
			}
		});  
	
	}

	this.setAction = function sendGoal(form) {
        var picture_qty = form == undefined ? 0 : form.device_timelapse_qty.value;
        var inter_picture_delay_s = form == undefined ? 0 : form.device_timelapse_frequency.value;
		var goal = new ROSLIB.Goal({
			actionClient : action,
			goalMessage : {
				picture_qty : parseFloat(picture_qty),
				inter_picture_delay_s : parseFloat(inter_picture_delay_s),
				mode : parseInt($('input[name=capture_type]:checked', 'form[name=timelapse]').val())
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

    this.setDownloadQty = function(qty) {
        var setting = new ROSLIB.Param({
            ros : ros,
            name : name + '/DownloadQty'
        });
        setting.set(parseInt(qty));
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

	this.saveSequenceShell = function(){
	// ----- save sequence shell ----- #jb
		var save = new ROSLIB.Service({
			ros : ros,
			name : '/' + name +'/save_config_shell',
			serviceType : 'camera_network_msgs/CommandOption'
		});
		//alert($('#shellCommand').val())
		var request = new ROSLIB.ServiceRequest({option: $('#shellCommand').val()});
		save.callService(request, function(result) {});
		//this.output = new this.getSequenceShell()
	}

	/** this.getSequenceShell = function(){
	// ----- save sequence shell ----- #jb
		var save = new ROSLIB.Service({
			ros : ros,
			name : '/' + name +'/get_config_shell',
			serviceType : 'std_srvs/Trigger'
		});
		//alert($('#shellCommand').val())
		var request = new ROSLIB.ServiceRequest({});
		save.callService(request, function(result) {});
	} **/
	
	this.shutdownDevice = function(){
		var shutdown = new ROSLIB.Service({
			ros : ros,
			name : '/' + name + '/shutdown_device',
			serviceType : 'camera_network_msgs/CommandOption'
		});
		var request = new ROSLIB.ServiceRequest({option:"-h"});
		shutdown.callService(request, function(result) {});
	}

	this.downloadDevice = function(){
		var download = new ROSLIB.Service({
			ros : ros,
			name : '/' + name + '/download_data',
			serviceType : 'camera_network_msgs/Uint32'
		});
		var request = new ROSLIB.ServiceRequest({integer:0});
		download.callService(request, function(result) {});
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
		var update_srv = new ROSLIB.Service({
			ros : ros,
			name : '/' + name + '/update_camera',
			serviceType : 'camera_network_msgs/InCameraData'
		});
		var request = new ROSLIB.ServiceRequest({
			iso : form.device_parameter_iso.value,
            aperture : form.device_parameter_aperture.value,
            shutterspeed : form.device_parameter_shutterspeed.value,
            imageformat : form.device_parameter_imageformat.value
		});
		update_srv.callService(request,function(result){});

        /* Apply the configuration to the camera */
		//_current_device.setAction();
        this.setDownloadQty(form.device_dowload_qty.value);
        this.setConfig();
	}

	this.getInformation = function(form){
		var save = new ROSLIB.Service({
			ros : ros,
			name : '/' + name +'/get_camera',
			serviceType : 'camera_network_msgs/OutCameraData'
		});

		var allrequest = new ROSLIB.ServiceRequest({getAllInformation:true});
		save.callService(allrequest, function(result) {
            var config = getConfiguration(result['iso']);
            setSelectOptions("#device_parameter_iso", config[1], config[0]);
            config = getConfiguration(result['aperture']);
            setSelectOptions("#device_parameter_aperture", config[1], config[0]);
            setSelectOptions("#device_sequence_aperture", config[1], config[0]);
            config = getConfiguration(result['shutterspeed']);
            setSelectOptions("#device_parameter_shutterspeed", config[1], config[0]);
            setSelectOptions("#device_sequence_shutterspeed", config[1], config[0]);
            config = getConfiguration(result['imageformat']);
            setSelectOptions("#device_parameter_imageformat", config[1], config[0]);
            $("#param_status").html('');
            $("#device_totalspace").text(result['totalSpace']);
            $("#device_freespace").text(result['freeSpace']);
            $("#device_freeimages").text(result['freeImages']);
		});
        param.get(function(value){
           $("#device_dowload_qty").val(value["DownloadQty"]);
        });
	}

	this.refresh = function() {
		$("#device_name").text(name);
        $("#device_ip").text(ip);
        $("#msg_output").text(output);
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

    this.setConfig = function(){
		var cal = new ROSLIB.Service({
			ros : ros,
			name : '/' + name + '/set_device_settings',
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

//   ----  Instances   -----

var _network_timelapse = new network_timelapse();
var _current_device;
var _device_list;
//var img = new Image;
//img.src = "http://" + ROS_MASTER + ":8181/stream?topic=/preview?width=640?height=480";

//   ---   GUI Functions   ----

function cleanParametersWidgets() {
    $("#device_parameter_iso option").remove()
    $("#device_parameter_aperture option").remove()
    $("#device_parameter_shutterspeed option").remove()
    $("#device_parameter_imageformat option").remove()
    $("#device_sequence_aperture option").remove()
    $("#device_sequence_shutterspeed option").remove()
    $("#device_sequence_imageformat option").remove()
}


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
    cleanParametersWidgets();
}


function refreshScreen(){
	if (_current_device == undefined){
		cleanDevicePage();
	}
     else{
          _current_device.refresh();
	}
}


function refreshDevices()
{
	IpList = new ROSLIB.Param({
		ros : ros,
		name : '/IP'
	});
	IpList.get(function(result) {
        _device_list = result;

        /* Update the list of devices */
        var device_list_text = 'Devices: ';
        $.each( _device_list, function( key, value ) {
            device_list_text += key + ", ";
        });

        $("#device_list").text(device_list_text.slice(0, -2));

        /* Update device select if we're in the device page */
        if ($("#deviceList").length != 0) {
            refreshSelect();
        }

        /* Only start the timer after a request is received */
        setTimeout(refreshDevices, 2000);
	});
}

/* Refreshes the device select in the device page. */
function refreshSelect()
{
    var select = $("#deviceList");
    var old_value = select.val();
    $("#deviceList option").remove();
    select.prop("disabled", false);

    $.each( _device_list, function( key, value ) {
        if ($("#deviceList option[value='" + value + "']").length == 0) {
            select.append($("<option></option>")
                    .attr("value", value)
                    .text(key));
        } else {
            console.log("Found duplicate IP addresses for devices!");
        }
    });

    /* Set currently selected device to previously selected one */
    if (_current_device != undefined) {
        $("#deviceList").val(_current_device.getIp());
    }

    /* Trigger onChange event */
    if (select.val() != old_value) {
        select.change();
    }
}

function noDeviceAlert()
{
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

function networkTimelapseEvent(form,isStart){
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

function saveSequenceShellEvent(form){
// --- save shell configuration --- #jb
	if(!noDeviceAlert()){
		_current_device.saveSequenceShell(form);
	}
}

function deviceTimelapseEvent(form, isStart){
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

function downloadDeviceEvent(form){
	if(!noDeviceAlert()){
		var r = confirm("Are you sure you want to download all pictures from device?");
		if (r == true) {
		    _current_device.downloadDevice();
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
        cleanParametersWidgets();
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
        $.blockUI({ message: '<img src="./media/loading.gif" style="vertical-align: middle; margin: 1em;" /> Connecting to ROS...' });
    }
    refreshDevices();
    if($("#preview_image").length != 0){
        initPreview();
    }
    var refreshScreenTimer = setInterval(refreshScreen, 1000);

});
