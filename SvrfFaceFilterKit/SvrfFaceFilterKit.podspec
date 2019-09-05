Pod::Spec.new do |s|
  s.name             = "SvrfFaceFilterKit"
  s.version          = "0.0.1"
  s.summary          = "The world's best worst face detection system."
  s.homepage         = "https://github.com/Svrf/face-landmarking-ios/tree/jesses-horrible-fixes/SvrfFaceFilterKit"
  s.license          = '¯\_(ツ)_/¯'
  s.author           = { "Jesse Boyes + a bunch of OSS" => "jesse@svrf.com" }
  s.source           = { :git => "https://github.com/Svrf/face-landmarking-ios/tree/jesses-horrible-fixes/SvrfFaceFilterKit", :tag => s.version }

  s.platform     = :ios, '11.0'
  s.requires_arc = true

  s.source_files = 'SvrfFaceFilterKit'

  s.frameworks = 'UIKit', 'ARKit'
  s.module_name = 'SvrfFaceFilterKit'
end

