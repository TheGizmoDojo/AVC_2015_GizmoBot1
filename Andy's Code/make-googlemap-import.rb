filename = ARGV[0]

coord = nil
n = 1


File.open(filename, "r").each_line do |line|
  data = line.chomp().split(",")
  #puts data[0]
  if data[0] == 'START'
    n = 1
  elsif data[0] == 'GPS'
    coord = data[2] + "," + data[1]
    puts coord.chomp + ",Point" + n.to_s
    n = n + 1
  end
end
