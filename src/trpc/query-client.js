"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
exports.createQueryClient = void 0;
var react_query_1 = require("@tanstack/react-query");
var superjson_1 = require("superjson");
var createQueryClient = function () {
    return new react_query_1.QueryClient({
        defaultOptions: {
            queries: {
                // With SSR, we usually want to set some default staleTime
                // above 0 to avoid refetching immediately on the client
                staleTime: 30 * 1000,
            },
            dehydrate: {
                serializeData: superjson_1.default.serialize,
                shouldDehydrateQuery: function (query) {
                    return (0, react_query_1.defaultShouldDehydrateQuery)(query) ||
                        query.state.status === "pending";
                },
            },
            hydrate: {
                deserializeData: superjson_1.default.deserialize,
            },
        },
    });
};
exports.createQueryClient = createQueryClient;
