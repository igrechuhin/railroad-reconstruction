# LaserScanner

module LaserScanner
  @debag=false

  def self.preferences
 

    @model = Sketchup.active_model
    bbox = @model.bounds
    @length = bbox.corner(7).y.to_f.to_m # (7 - right back top corner of the bounding box)
    
    prompts = [ "The length of the path trains (m): ","The height of the attachment point of the camera (m): ", "d(fi)  (in degrees)","d(y)  (in meters)"]
    defaults = ["#{@length}","2","1","0.1"]
    list = ["","","",""]
    input = UI.inputbox prompts, defaults, list, "Railway Laser Scanner"
    return unless input
    @length = input[0].to_f.m.to_f
    @laser_height = input[1].to_f.m.to_f
    @d_fi = input[2].to_f.degrees
    @d_y = input[3].to_f.m.to_f

  
    title = @model.title
    @path_to_save_to = UI.savepanel "Save scan data", "c:\\", "#{title}.csv"
  end#def

  def self.scan
    return unless @path_to_save_to
    puts "Cartesian coordinate system (X,Y,Z)"
    p "@laser_height #{@laser_height}" if @debag
    p "@length #{@length}" if @debag
    p "@d_y #{@d_y}" if @debag
    p "(@length/@d_y).floor #{(@length/@d_y).floor}" if @debag
    p "@d_fi #{@d_fi}" if @debag
    p "(Math::PI/@d_fi).floor #{(Math::PI/@d_fi).floor}" if @debag
    puts ">> Scanning started. #{((@length/@d_y).floor+1)*((Math::PI/@d_fi).floor+1)} points. It may take a few minutes..."
    f = File.new( @path_to_save_to,  "w+")
    l_position = Geom::Point3d.new
    l_vector = Geom::Vector3d.new
    
    l_position.x= 0
    l_position.z= @laser_height
    l_vector.y= 0
    for i in 0..(@length/@d_y).floor 
      l_position.y= i*@d_y
      for j in 0..(Math::PI/@d_fi).floor
        fi = (-1.0/2)*Math::PI+j*@d_fi
        l_vector.x= Math::cos(fi)
        l_vector.z= Math::sin(fi)
        p "_____________#{i}/#{j}" if @debag
        p l_position.to_s if @debag
        p l_vector.to_s if @debag
        item = @model.raytest([l_position, l_vector])
        unless item.nil? 
          f.write("#{item[0].x.to_f.to_m},#{item[0].y.to_f.to_m},#{item[0].z.to_f.to_m}\n")
        end#unless
     end#for
    end#for
   f.close
   puts ">> Scan completed!"
 
  end#def

  def self.start
    Sketchup.send_action "showRubyPanel:"
      Sketchup.active_model.start_operation "Railway Laser Scanner"
      self.preferences
      self.scan
      Sketchup.active_model.commit_operation
  end#def


#Menus and toolbars
file = File.basename(__FILE__)
  unless file_loaded?(file)
    UI.menu("Plugins").add_item("Railway Laser Scanner") {self.start}
  end#unless

end#module