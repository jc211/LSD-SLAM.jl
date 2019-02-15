using GLFW
using ModernGL




function main()

	positions = [-0.5f0, -0.5f0, 0.0f0, 0.5f0, 0.5f0, -0.5f0]
	buffer::Ref{UInt32} = 0
	glGenBuffers(1, buffer)
	glBindBuffer(GL_ARRAY_BUFFER, buffer.x)
	glBufferData(GL_ARRAY_BUFFER, 6*sizeof(eltype(positions)), positions, GL_STATIC_DRAW)
	glEnableVertexAttribArray(0)
	glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2*sizeof(eltype(positions)), Ref(0))

	# Loop until the user closes the window
	while !GLFW.WindowShouldClose(window)

		glClear(GL_COLOR_BUFFER_BIT)
		# Render here
		glDrawArrays(GL_TRIANGLES, 0, 3)
		# Swap front and back buffers
		GLFW.SwapBuffers(window)

		# Poll for and process events
		GLFW.PollEvents()
	end


end
# Create a window and its OpenGL context
window = GLFW.CreateWindow(640, 480, "GLFW.jl")
# Make the window's context current
GLFW.MakeContextCurrent(window)
main()
GLFW.DestroyWindow(window)
