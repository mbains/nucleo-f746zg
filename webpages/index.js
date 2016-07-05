$(document).ready(function () {
    console.log("hello");
    var do_request = function () {
        $.get('/irq', function (result) {
            console.log(result);
            setTimeout(do_request, 0);
            $("#mcu-temp").find("b").html(result);
        });
    };
    do_request();


});
  
