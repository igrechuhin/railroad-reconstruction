# LaserScanner

module LaserScanner
  def self.preferences
 

    @model = Sketchup.active_model
    bbox = @model.bounds
    @length = bbox.corner(7).y.to_f.to_m # (7 - right back top corner of the bounding box)
    prompts = [ "Длина пути пройденного поездом (в метрах): ","Высота точки подвеса камеры (в метрах): ", "Шаг d(fi) по углу вращения (в градусах)","Шаг поступательного движения поезда d(y)  (в метр)","Угол между плоскостью вращения лазера и осью ОУ (in degrees): ","Среднеквадрати́чное отклоне́ние лазерного луча (в градусах): "]
    defaults = ["#{@length}","2","1","0.1","90", "0.01"]
    list = ["","","","","",""]
    input = UI.inputbox prompts, defaults, list, "Railway Laser Scanner"
    return unless input
    @length = input[0].to_f.m.to_f
    @laser_height = input[1].to_f.m.to_f
    @d_fi = input[2].to_f.degrees
    @d_y = input[3].to_f.m.to_f
    @angle = input[4].to_f.degrees
    @SD = input[5].to_f.degrees
    title = @model.title
    @path_to_save_to = UI.savepanel "Save scan data", "c:\\", "#{title}.csv"
  end#def

  def self.scan
    return unless @path_to_save_to
    puts "Cartesian coordinate system (X,Y,Z)"
    f = File.new( @path_to_save_to,  "w+")
    l_position = Geom::Point3d.new
    l_vector = Geom::Vector3d.new
    
    l_position.x= 0
    l_position.z= @laser_height
    l_vector.y= 0
    if @angle == 90.degrees
      puts ">> Scanning started. #{((@length/@d_y).floor+1)*((2*Math::PI/@d_fi).floor+1)} points. It may take a few minutes..."
      for i in 0..(@length/@d_y).floor 
      l_position.y= i*@d_y
      for j in 0..(2*Math::PI/@d_fi).floor
        fi = (-1.0/2)*Math::PI+j*@d_fi
        l_vector.x= Math.cos(fi + self.rand_normal)
        l_vector.z= Math.sin(fi + self.rand_normal)
        l_vector.y= Math.cos(@angle + self.rand_normal)
        item = @model.raytest([l_position, l_vector])
        unless item.nil? 
          f.write("#{item[0].x.to_f.to_m},#{item[0].y.to_f.to_m},#{item[0].z.to_f.to_m}\n")
        end#unless
      end#for
      end#for
    else
      puts ">> Scanning started. #{2*((@length/@d_y).floor+1)*((2*Math::PI/@d_fi).floor+1)} points. It may take a few minutes..."
      for i in 0..(@length/@d_y).floor 
      l_position.y= i*@d_y
      for j in 0..(2*Math::PI/@d_fi).floor
        fi = (-1.0/2)*Math::PI+j*@d_fi
        l_vector.x= Math.cos(fi + self.rand_normal)
        l_vector.z= Math.sin(fi + self.rand_normal)
        l_vector.y= Math.cos(@angle + self.rand_normal)
        item = @model.raytest([l_position, l_vector])
        unless item.nil? 
          f.write("#{item[0].x.to_f.to_m},#{item[0].y.to_f.to_m},#{item[0].z.to_f.to_m}\n")
        end#unless
        l_vector.y= -1*Math.cos(@angle + self.rand_normal)
        item = @model.raytest([l_position, l_vector])
        unless item.nil? 
          f.write("#{item[0].x.to_f.to_m},#{item[0].y.to_f.to_m},#{item[0].z.to_f.to_m}\n")
        end#unless
      end#for
      end#for
    end#if
   f.close
   puts ">> Scan completed!"
   @path_to_save_to = nil;
  end#def

  def self.rand_normal
    r = Random.new
    x = r.rand * (-1)** r.rand(100)
    y = r.rand * (-1)** r.rand(100)
    while x**2+y**2 > 1 do
      x = r.rand * (-1)** r.rand(100)
      y = r.rand * (-1)** r.rand(100)
    end
    s = x**2+y**2
    z = x*Math.sqrt(-2*Math.log(s)/s)
    return Math.sqrt(@SD)*z
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