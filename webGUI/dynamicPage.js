$(function(){

    // Highlights the nav button corresponding to the current page
    $.each( $("nav").children(), function(index, value) {
        if (window.location.pathname.split("/").pop() == value.pathname.split("/").pop()) {
            $(value).addClass("current");
        }
    });

});
