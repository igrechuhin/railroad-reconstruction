# CameraView

# Author:	Abramenko Alexander, mr.abramenko@gmail.com

# Usage
#	Menu: Plugins
#		Railway Camera View: Save image
#

#Load the normal support files
require 'sketchup.rb'
require 'extensions.rb'

module CameraView

PLUGIN_ROOT = File.dirname(__FILE__) unless defined?(self::PLUGIN_ROOT)

ex = SketchupExtension.new("Railway Camera View", File.join(PLUGIN_ROOT, "\\CameraView\\main.rb"))
ex.description = "Save image"
ex.version = "0.1.0"
ex.copyright = "© 2014"
ex.creator = "Abramenko Alexander"

Sketchup.register_extension ex, true

end#module