# CameraView

module CameraView

  def self.preferences
    @model = Sketchup.active_model
    bbox = @model.bounds
    length = bbox.corner(7).y.to_f.to_m # (7 - right back top corner of the bounding box)
      
    prompts = ["Высота точки подвеса камеры (в метрах): ","Длина пути пройденного поездом (в метрах): ", "Кол-во фотографий: ","Ширина изображения в пикселях (max 16000): ","Высота изображения в пикселях (max 16000): "]
    defaults = ["2","#{length}", "7","1600","900"]
    input = UI.inputbox prompts, defaults, "Railway Camera View"
    return unless input
    @cam_height = input[0].to_f.m.to_f
    @path_length = input[1].to_f.m.to_f
    @num_photos = input[2].to_i
    @img_width = input[3].to_i
    @img_height= input[4].to_i

    
    title = @model.title
    @path_to_save_to = UI.savepanel("Save images", "c:\\", "#{title}.png")
    
 	end#def

  def self.save_images
    return unless @path_to_save_to
     dirname = File.dirname(@path_to_save_to)
     basename = File.basename(@path_to_save_to, ".png")
     keys = {
             :filename => File.join("c:/", dirname, basename + "#{}" + ".png"),
             :width => @img_width,
             :height => @img_height,
             :antialias => false,
             :compression => 1,
             :transparent => false
           }
     view = @model.active_view
     original_cam = view.camera
     # Create a camera from scratch with an "eye" position in
     # x, y, z coordinates, a "target" position that
     # defines what to look at, and an "up" vector.
     eye = Geom::Point3d.new(0,0,0)
     target = Geom::Point3d.new(1,0,0)
     up = Geom::Vector3d.new(0,0,1)
     camera_train = Sketchup::Camera.new eye, target, up

     puts ">> Save #{@num_photos} images..."
    for i in 0...@num_photos
          eye = Geom::Point3d.new(0,i*@path_length/(@num_photos-1),@cam_height)
          target = Geom::Point3d.new(1,i*@path_length/(@num_photos-1),@cam_height)
               
          camera_train.set eye, target, up
          view.camera= camera_train
          keys[:filename]= File.join(dirname, basename + "(#{i})" + ".png")
          puts File.join( dirname, basename + "(#{i})" + ".png")

          view.write_image keys
     end#for
     view.camera= original_cam

  end#def

  def self.start
    Sketchup.send_action "showRubyPanel:"
    Sketchup.active_model.start_operation "Railway Camera View"
    self.preferences
    self.save_images
    Sketchup.active_model.commit_operation
  end#def

#Menus and toolbars
file = File.basename(__FILE__)
  unless file_loaded?(file)
    UI.menu("Plugins").add_item("Railway Camera View") {self.start}
  end#unless

end#module