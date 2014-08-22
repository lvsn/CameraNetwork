
$(function(){

    var newHash = '',
        $mainContent = $("#main-content");

    $("nav").delegate("a","click",function(){
        window.location.hash = $(this).attr("href");
        $('.current').removeClass("current");
        $(this).addClass("current");
        return false;
    });

    $(window).bind('hashchange',function(){
        newHash = window.location.hash.substring(1);
        $mainContent.load(newHash + " #sub-content");
    });

});