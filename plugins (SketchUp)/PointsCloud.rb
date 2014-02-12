# PointsCloud

# Author:	Abramenko Alexander, mr.abramenko@gmail.com

# Usage
#	Menu: Plugins
#		Railway Points Cloud:	Import points from a CSV file
#

#Load the normal support files
require 'sketchup.rb'
require 'extensions.rb'

module PointsCloud

PLUGIN_ROOT = File.dirname(__FILE__) unless defined?(self::PLUGIN_ROOT)

ex = SketchupExtension.new("Railway Points Cloud", File.join(PLUGIN_ROOT, "\\PointsCloud\\main.rb"))
ex.description = "Import points from a CSV file."
ex.version = "0.1.0"
ex.copyright = "© 2014"
ex.creator = "Abramenko Alexander"

Sketchup.register_extension ex, true

end#module