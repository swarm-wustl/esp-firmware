-- Project-local Neovim config for swarm.
--
-- Points clangd at scripts/clangd.sh, which runs the language server inside the
-- dev container so it reads the native ESP-IDF v5.5 headers (the host needs no
-- IDF install or symlinks). Scoped to THIS project only.
--
-- Requires exrc to be enabled in your global config (one-time, opt-in):
--     vim.o.exrc = true
-- Neovim will prompt once to trust this file (vim.secure). Open Neovim from the
-- repo root so exrc picks it up.

local root = vim.fn.fnamemodify(debug.getinfo(1, "S").source:sub(2), ":p:h")
local cmd = { root .. "/scripts/clangd.sh" }

if vim.lsp.config then
  -- Neovim 0.11+: extend the clangd config shared by vim.lsp.enable and
  -- nvim-lspconfig so only the launch command changes.
  vim.lsp.config("clangd", { cmd = cmd })
  vim.lsp.enable("clangd")
else
  -- Older setups using nvim-lspconfig directly.
  local ok, lspconfig = pcall(require, "lspconfig")
  if ok then
    lspconfig.clangd.setup({ cmd = cmd })
  end
end
