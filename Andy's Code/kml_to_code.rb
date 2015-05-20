filename = ARGV[0]

puts "Location WAYPOINT[] = {"
File.open(filename, "r").each_line do |line|
	line = line.chomp()
 	if line['coordinate']
		pos1 = line.index(">")
		pos2 = line.index(",", pos1)
		pos3 = line.index(",", pos2+1)
		longitude = line[(pos1+1)..(pos2-1)]
		latitude = line[(pos2+1)..(pos3-1)]
		puts "  new Location(#{latitude}, #{longitude}),"
	end
end
puts "}"


