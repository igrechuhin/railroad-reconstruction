# PointsCloud

module PointsCloud

  def self.preferences
    @model = Sketchup.active_model
    title = @model.title
 	  @path = UI.openpanel "Open", "c:\\", "#{title}.csv"
 	end#def

  def self.import
    return unless @path
    puts "Counting the number of points..."
 	  f=File.new(@path,"r")
	  all = 0
 	  while f.gets
    	all += 1
 	  end#while
   	f.close

 	  puts ">> Import #{all} points.It may take a few minutes..."
 	  i = 0
    entities = @model.active_entities
    f=File.new(@path,"r")
 	  while line = f.gets
    	xyz = line.chomp.split(',')
   		entities.add_cpoint Geom::Point3d.new (xyz[0].to_f.m,xyz[1].to_f.m,xyz[2].to_f.m)
    	i +=1
    end#while
    f.close
     puts ">> Import completed!"
  end#def

  def self.start
    Sketchup.send_action "showRubyPanel:"
      Sketchup.active_model.start_operation "Railway Points Cloud"
      self.preferences
      self.import
      Sketchup.active_model.commit_operation
  end#def

#Menus and toolbars
file = File.basename(__FILE__)
  unless file_loaded?(file)
    UI.menu("Plugins").add_item("Railway Points Cloud") {self.start}
  end#unless

end#module












