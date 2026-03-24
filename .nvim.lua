vim.lsp.config("clangd", {
	cmd = {
		"/Users/prestonmeek/.espressif/tools/esp-clang/16.0.1-fe4f10a809/esp-clang/bin/clangd",
		"--query-driver=/Users/prestonmeek/.espressif/tools/xtensa-esp-elf/**/bin/xtensa-esp-elf-*",
	},
})

-- Restart the client so it picks up the new config
vim.lsp.stop_client(vim.lsp.get_clients({ name = "clangd" }))
