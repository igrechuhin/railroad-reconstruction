# LaserScanner

# Author:	Abramenko Alexander, mr.abramenko@gmail.com

# Usage
#	Menu: Plugins
#		Railway Laser Scanner:	Emulation of railway laser scanning.
#

#Load the normal support files
require 'sketchup.rb'
require 'extensions.rb'

module LaserScanner

PLUGIN_ROOT = File.dirname(__FILE__) unless defined?(self::PLUGIN_ROOT)

ex = SketchupExtension.new("Railway Laser Scanner", File.join(PLUGIN_ROOT, "\\LaserScanner\\main.rb"))
ex.description = "Emulation of railway laser scanning."
ex.version = "0.1.0"
ex.copyright = "© 2014"
ex.creator = "Abramenko Alexander"

Sketchup.register_extension ex, true

end#module