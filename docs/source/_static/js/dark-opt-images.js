$(document).ready(function () {
  const DARK_OPT_CLASS = "has-dark-opt";
  function updateImgSrcs() {
    const isDark = $("body").attr("data-md-color-scheme") === "slate";
    $("img." + DARK_OPT_CLASS).each(function () {
      var src = isDark ? $(this).attr("dark-src") : $(this).attr("light-src");
      $(this).attr("src", src);
    });
  }

  $("img." + DARK_OPT_CLASS).each(function () {
    const src = $(this).attr("src");
    const components = src.split(".");
    const hasExtension = components.length > 1;
    const extension = components.pop();

    const dark_src = hasExtension
      ? components.join(".") + "-dark." + extension
      : src + "-dark";

    $(this).attr("dark-src", dark_src);
    $(this).attr("light-src", src);
  });

  // use MutationObserver to trigger update on color scheme change
  // ie, dark/light mode toggle
  var observer = new MutationObserver(updateImgSrcs);
  observer.observe(document.querySelector("body"), {
    attributes: true,
    attributeFilter: ["data-md-color-scheme"],
  });
  updateImgSrcs();
});
