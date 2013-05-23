moveit_web
==========

Web-based tools using the MoveIt planning framework


Web-UI
------

To run the web UI alone (without connecting to a backend), you'll need to make sure the app is configured for debuggin. This involves editing the `webui/index.html`, where app initialization occurs:

```html
        <script>
        ...
        config.debug = true;
	    app.init(config);
        ...
        </script>
```

After this, navigate to the webui directory and run `python2 -m SimpleHTTPServer` for a fast and cheap file server. Navigating your WebGL enabled browser to `localhost:8000` will give you access to the web UI.