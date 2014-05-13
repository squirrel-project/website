({
    appDir: "js/",
    baseUrl: "./",
    dir: "build",
    modules: [{
        name: "application"
    }],
    map: {
      '*': { 'jquery': 'jquery-private' },
      'jquery-private': { 'jquery': 'jquery' }
    },
    paths: {
        jquery:     'jquery-1.11.0.min',
        hbs: '../vendor/hbs/hbs',
        templates: '../templates/'
    }
})
